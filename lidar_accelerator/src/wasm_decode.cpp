#include "lidar_accelerator/wasm_decode.hpp"
#include "lidar_accelerator/processing.hpp"

#include <stdexcept>
#include <string>

#if defined(GO2_WASMTIME_C_API)
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <wasmtime.h>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iterator>
#include <mutex>
#include <vector>
#endif

namespace lidar_accelerator {

std::vector<float> decode_and_process(
    const uint8_t* compressed,
    std::size_t compressed_len,
    float res,
    const float origin[3],
    float intense_limiter,
    bool deduplicate,
    int downsample_step,
    int max_points,
    std::size_t* out_points) {
#if !defined(GO2_WASMTIME_C_API)
  throw std::runtime_error("wasmtime C API is not available in this build");
#else
  if (!compressed || compressed_len == 0) {
    throw std::runtime_error("compressed input is empty");
  }

  struct GlobalCtx {
    wasm_engine_t* engine = nullptr;
    wasmtime_module_t* module = nullptr;
  };

  struct ThreadCtx {
    wasmtime_store_t* store = nullptr;
    wasmtime_instance_t instance{};
    bool instantiated = false;

    bool exports_cached = false;
    wasmtime_extern_t gen_ex{};
    wasmtime_extern_t malloc_ex{};
    wasmtime_extern_t mem_ex{};

    bool buffers_allocated = false;
    int32_t input = 0;
    int32_t decompressBuffer = 0;
    int32_t positions_ptr = 0;
    int32_t uvs_ptr = 0;
    int32_t indices_ptr = 0;
    int32_t decompressedSize_ptr = 0;
    int32_t faceCount_ptr = 0;
    int32_t pointCount_ptr = 0;
  };

  static std::mutex g_mu;
  static GlobalCtx g;
  static thread_local ThreadCtx t;

  auto error_to_string = [](wasmtime_error_t* err) -> std::string {
    wasm_name_t msg;
    wasmtime_error_message(err, &msg);
    std::string s(msg.data, msg.size);
    wasm_name_delete(&msg);
    wasmtime_error_delete(err);
    return s;
  };

  auto trap_to_string = [](wasm_trap_t* trap) -> std::string {
    wasm_message_t msg;
    wasm_trap_message(trap, &msg);
    std::string s(msg.data, msg.size);
    wasm_byte_vec_delete(&msg);
    wasm_trap_delete(trap);
    return s;
  };

  auto get_export = [](wasmtime_context_t* ctx,
                       const wasmtime_instance_t* inst,
                       const char* name) -> wasmtime_extern_t {
    wasmtime_extern_t item;
    bool ok = wasmtime_instance_export_get(ctx, inst, name, std::strlen(name), &item);
    if (!ok) {
      throw std::runtime_error(std::string("missing wasm export: ") + name);
    }
    return item;
  };

  auto host_adjust_memory_size = [](void*, wasmtime_caller_t* caller, const wasmtime_val_t* args,
                                   std::size_t, wasmtime_val_t* results, std::size_t) -> wasm_trap_t* {
    (void)args;
    wasmtime_extern_t mem_extern;
    bool ok = wasmtime_caller_export_get(caller, "c", 1, &mem_extern);
    if (!ok || mem_extern.kind != WASMTIME_EXTERN_MEMORY) {
      return nullptr;
    }
    wasmtime_context_t* ctx = wasmtime_caller_context(caller);
    std::size_t sz = wasmtime_memory_data_size(ctx, &mem_extern.of.memory);
    results[0].kind = WASMTIME_I32;
    results[0].of.i32 = static_cast<int32_t>(sz);
    return nullptr;
  };

  auto host_copy_memory_region = [](void*, wasmtime_caller_t* caller, const wasmtime_val_t* args,
                                   std::size_t, wasmtime_val_t*, std::size_t) -> wasm_trap_t* {
    wasmtime_extern_t mem_extern;
    bool ok = wasmtime_caller_export_get(caller, "c", 1, &mem_extern);
    if (!ok || mem_extern.kind != WASMTIME_EXTERN_MEMORY) {
      return nullptr;
    }
    wasmtime_context_t* ctx = wasmtime_caller_context(caller);
    uint8_t* data = wasmtime_memory_data(ctx, &mem_extern.of.memory);
    std::size_t sz = wasmtime_memory_data_size(ctx, &mem_extern.of.memory);

    const int32_t t = args[0].of.i32;
    const int32_t n = args[1].of.i32;
    const int32_t a = args[2].of.i32;
    if (t < 0 || n < 0 || a < 0) {
      return nullptr;
    }
    const std::size_t dst = static_cast<std::size_t>(t);
    const std::size_t src = static_cast<std::size_t>(n);
    const std::size_t len = static_cast<std::size_t>(a);
    if (dst + len > sz || src + len > sz) {
      return nullptr;
    }
    std::memmove(data + dst, data + src, len);
    return nullptr;
  };

  wasmtime_context_t* ctx = nullptr;

  auto call_i32 = [&](const wasmtime_func_t* fn, int32_t arg0) -> int32_t {
    wasmtime_val_t args[1];
    args[0].kind = WASMTIME_I32;
    args[0].of.i32 = arg0;
    wasmtime_val_t results[1];
    results[0].kind = WASMTIME_I32;
    wasm_trap_t* trap = nullptr;
    wasmtime_error_t* err = wasmtime_func_call(ctx, fn, args, 1, results, 1, &trap);
    if (err) {
      throw std::runtime_error(error_to_string(err));
    }
    if (trap) {
      throw std::runtime_error(trap_to_string(trap));
    }
    return results[0].of.i32;
  };

  {
    std::lock_guard<std::mutex> lk(g_mu);
    if (!g.engine) {
      std::string wasm_path = ament_index_cpp::get_package_share_directory("go2_robot_sdk") +
                              "/external_lib/libvoxel.wasm";

      g.engine = wasm_engine_new();

      std::ifstream f(wasm_path, std::ios::binary);
      if (!f) {
        throw std::runtime_error(std::string("failed to open wasm module: ") + wasm_path);
      }
      std::vector<uint8_t> wasm_bytes((std::istreambuf_iterator<char>(f)),
                                      std::istreambuf_iterator<char>());
      if (wasm_bytes.empty()) {
        throw std::runtime_error(std::string("wasm module is empty: ") + wasm_path);
      }

      wasmtime_error_t* err = wasmtime_module_new(
          g.engine, wasm_bytes.data(), wasm_bytes.size(), &g.module);
      if (err) {
        throw std::runtime_error(error_to_string(err));
      }
    }
  }

  if (!t.store) {
    t.store = wasmtime_store_new(g.engine, nullptr, nullptr);
  }

  ctx = wasmtime_store_context(t.store);

  if (!t.instantiated) {
      wasm_valtype_t* params_a[1] = {wasm_valtype_new_i32()};
      wasm_valtype_t* results_a[1] = {wasm_valtype_new_i32()};

      wasm_valtype_vec_t params_a_vec;
      wasm_valtype_vec_t results_a_vec;
      wasm_valtype_vec_new(&params_a_vec, 1, params_a);
      wasm_valtype_vec_new(&results_a_vec, 1, results_a);
      wasm_functype_t* ft_a = wasm_functype_new(&params_a_vec, &results_a_vec);

      wasmtime_func_t fn_a;
      wasmtime_func_new(ctx, ft_a, host_adjust_memory_size, nullptr, nullptr, &fn_a);

      wasm_valtype_t* params_b[3] = {
          wasm_valtype_new_i32(), wasm_valtype_new_i32(), wasm_valtype_new_i32()};

      wasm_valtype_vec_t params_b_vec;
      wasm_valtype_vec_t results_b_vec;
      wasm_valtype_vec_new(&params_b_vec, 3, params_b);
      wasm_valtype_vec_new_empty(&results_b_vec);
      wasm_functype_t* ft_b = wasm_functype_new(&params_b_vec, &results_b_vec);

      wasmtime_func_t fn_b;
      wasmtime_func_new(ctx, ft_b, host_copy_memory_region, nullptr, nullptr, &fn_b);

      wasmtime_extern_t imports[2];
      imports[0].kind = WASMTIME_EXTERN_FUNC;
      imports[0].of.func = fn_a;
      imports[1].kind = WASMTIME_EXTERN_FUNC;
      imports[1].of.func = fn_b;

      wasm_trap_t* trap = nullptr;
      wasmtime_error_t* err =
          wasmtime_instance_new(ctx, g.module, imports, 2, &t.instance, &trap);
      if (err) {
        throw std::runtime_error(error_to_string(err));
      }
      if (trap) {
        throw std::runtime_error(trap_to_string(trap));
      }

    t.instantiated = true;
  }

  if (!t.exports_cached) {
    t.gen_ex = get_export(ctx, &t.instance, "e");
    t.malloc_ex = get_export(ctx, &t.instance, "f");
    t.mem_ex = get_export(ctx, &t.instance, "c");

    if (t.gen_ex.kind != WASMTIME_EXTERN_FUNC || t.malloc_ex.kind != WASMTIME_EXTERN_FUNC ||
        t.mem_ex.kind != WASMTIME_EXTERN_MEMORY) {
      throw std::runtime_error("unexpected export kinds");
    }

    t.exports_cached = true;
  }

  if (!t.buffers_allocated) {
    t.input = call_i32(&t.malloc_ex.of.func, 61440);
    t.decompressBuffer = call_i32(&t.malloc_ex.of.func, 80000);
    t.positions_ptr = call_i32(&t.malloc_ex.of.func, 2880000);
    t.uvs_ptr = call_i32(&t.malloc_ex.of.func, 1920000);
    t.indices_ptr = call_i32(&t.malloc_ex.of.func, 5760000);
    t.decompressedSize_ptr = call_i32(&t.malloc_ex.of.func, 4);
    t.faceCount_ptr = call_i32(&t.malloc_ex.of.func, 4);
    t.pointCount_ptr = call_i32(&t.malloc_ex.of.func, 4);

    t.buffers_allocated = true;
  }

  const wasmtime_extern_t gen_ex = t.gen_ex;
  const wasmtime_extern_t mem_ex = t.mem_ex;

  const int32_t input = t.input;
  const int32_t decompressBuffer = t.decompressBuffer;
  const int32_t positions_ptr = t.positions_ptr;
  const int32_t uvs_ptr = t.uvs_ptr;
  const int32_t indices_ptr = t.indices_ptr;
  const int32_t decompressedSize_ptr = t.decompressedSize_ptr;
  const int32_t faceCount_ptr = t.faceCount_ptr;
  const int32_t pointCount_ptr = t.pointCount_ptr;

  if (compressed_len > 61440) {
    throw std::runtime_error("compressed input too large");
  }

  uint8_t* mem = wasmtime_memory_data(ctx, &mem_ex.of.memory);
  std::size_t mem_sz = wasmtime_memory_data_size(ctx, &mem_ex.of.memory);

  if (static_cast<std::size_t>(input) + compressed_len > mem_sz) {
    throw std::runtime_error("compressed input too large");
  }
  std::memcpy(mem + static_cast<std::size_t>(input), compressed, compressed_len);

  const int32_t some_v = static_cast<int32_t>(std::floor(origin[2] / res));

  wasmtime_val_t args[11];
  auto set_i32 = [&](int idx, int32_t v) {
    args[idx].kind = WASMTIME_I32;
    args[idx].of.i32 = v;
  };
  set_i32(0, input);
  set_i32(1, static_cast<int32_t>(compressed_len));
  set_i32(2, 80000);
  set_i32(3, decompressBuffer);
  set_i32(4, decompressedSize_ptr);
  set_i32(5, positions_ptr);
  set_i32(6, uvs_ptr);
  set_i32(7, indices_ptr);
  set_i32(8, faceCount_ptr);
  set_i32(9, pointCount_ptr);
  set_i32(10, some_v);

  wasmtime_val_t gen_results[1];
  gen_results[0].kind = WASMTIME_I32;
  wasm_trap_t* trap = nullptr;
  wasmtime_error_t* err = wasmtime_func_call(ctx, &gen_ex.of.func, args, 11, gen_results, 1, &trap);
  if (err) {
    throw std::runtime_error(error_to_string(err));
  }
  if (trap) {
    throw std::runtime_error(trap_to_string(trap));
  }

  mem = wasmtime_memory_data(ctx, &mem_ex.of.memory);
  mem_sz = wasmtime_memory_data_size(ctx, &mem_ex.of.memory);

  auto read_i32 = [&](int32_t addr) -> int32_t {
    if (addr < 0 || static_cast<std::size_t>(addr + 4) > mem_sz) {
      throw std::runtime_error("read_i32 out of bounds");
    }
    int32_t v;
    std::memcpy(&v, mem + static_cast<std::size_t>(addr), sizeof(int32_t));
    return v;
  };

  const int32_t face_count = read_i32(faceCount_ptr);
  (void)read_i32(pointCount_ptr);

  const std::size_t u = static_cast<std::size_t>(face_count);
  const std::size_t pos_len = u * 12;
  const std::size_t uvs_len = u * 8;

  if (static_cast<std::size_t>(positions_ptr) + pos_len > mem_sz ||
      static_cast<std::size_t>(uvs_ptr) + uvs_len > mem_sz) {
    throw std::runtime_error("decoded buffers out of bounds");
  }

  std::size_t out_n = 0;
  std::vector<float> out = lidar_accelerator::process_u8_to_xyzi_f32(
      mem + static_cast<std::size_t>(positions_ptr),
      pos_len,
      mem + static_cast<std::size_t>(uvs_ptr),
      uvs_len,
      res,
      origin,
      intense_limiter,
      deduplicate,
      downsample_step,
      max_points,
      &out_n);

  if (out_points) {
    *out_points = out_n;
  }

  return out;
#endif
}

}  // namespace lidar_accelerator
