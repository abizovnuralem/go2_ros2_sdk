use std::collections::HashMap;
use std::env;
use std::io::Write;
use std::sync::Arc;

use anyhow::{anyhow, Result};
use base64::encode as base64_encode;
use chrono::Utc;
use clap::{AppSettings, Arg, Command};
use dotenv::dotenv;
use md5;
use rand::Rng;
use reqwest::Client;
use serde::Serialize;
use serde_json::{json, Value};
use tokio::net::UdpSocket;
use tokio::sync::Mutex;
use tokio::time::Duration;
use webrtc::api::interceptor_registry::register_default_interceptors;
use webrtc::api::media_engine::{MediaEngine, MIME_TYPE_H264, MIME_TYPE_OPUS};
use webrtc::api::APIBuilder;
use webrtc::data_channel::data_channel_message::DataChannelMessage;
use webrtc::ice_transport::ice_server::RTCIceServer;
use webrtc::interceptor::registry::Registry;
use webrtc::peer_connection::configuration::RTCConfiguration;
use webrtc::peer_connection::peer_connection_state::RTCPeerConnectionState;
use webrtc::peer_connection::sdp::session_description::RTCSessionDescription;
use webrtc::rtcp::payload_feedbacks::picture_loss_indication::PictureLossIndication;
use webrtc::rtp_transceiver::rtp_codec::{
    RTCRtpCodecCapability, RTCRtpCodecParameters, RTPCodecType,
};
use webrtc::rtp_transceiver::rtp_transceiver_direction::RTCRtpTransceiverDirection;
use webrtc::rtp_transceiver::RTCRtpTransceiverInit;
use webrtc::util::{Conn, Marshal};

#[derive(Serialize)]
struct PostData {
    sdp: String,
    id: String,
    #[serde(rename = "type")]
    type_field: String,
    token: String,
}

#[derive(Clone)]
struct UdpConn {
    conn: Arc<dyn Conn + Send + Sync>,
    payload_type: u8,
}

pub fn hex_to_base64(hex_str: &str) -> String {
    // Convert hex string to bytes
    let bytes_array = hex::decode(hex_str).expect("Decoding failed");
    println!("Bytes array: {:?}", bytes_array);
    // Encode the bytes to Base64 and return as a string
    base64_encode(bytes_array)
}

pub fn encrypt_key(key: &str) -> String {
    // Append the prefix to the key
    let prefixed_key = format!("UnitreeGo2_{}", key.replace("\"", ""));
    // Encrypt the key using MD5 and convert to hex string
    let encrypted = encrypt_by_md5(&prefixed_key);
    hex_to_base64(&encrypted)
}

pub fn encrypt_by_md5(input_str: &str) -> String {
    // Complete the hash and take the result
    let result = md5::compute(input_str.as_bytes());
    // Return the hex digest of the hash
    format!("{:x}", result)
}

pub fn generate_id() -> i64 {
    let timestamp = Utc::now().timestamp_millis() as i64 % 2147483648;
    let random_add: i64 = rand::thread_rng().gen_range(0..1000);
    timestamp + random_add
}

async fn post_offer(ip: &str, token: &str, offer_sdp: &str) -> Result<Option<Value>> {
    let url = format!("http://{}:8081/offer", ip);
    let client = Client::new();
    let data = PostData {
        sdp: offer_sdp.to_string(),
        id: "STA_localNetwork".to_string(),
        type_field: "offer".to_string(),
        token: token.to_string(),
    };

    let resp = client
        .post(&url)
        .json(&data)
        .header("content-type", "application/json")
        .send()
        .await?;

    match resp.status().is_success() {
        true => {
            let json: Value = resp.json().await?;
            Ok(Some(json))
        }
        false => {
            println!("Failed to get answer from server");
            Ok(None)
        }
    }
}

#[tokio::main]
async fn main() -> Result<()> {
    dotenv().ok();

    let robot_ip_env = env::var("GO2_IP").unwrap_or_else(|_| "".to_string());
    let robot_token_default = env::var("GO2_TOKEN").unwrap_or_else(|_| "".to_string());

    let mut app = Command::new("go2webrtc-rc")
        .version("0.1.0")
        .about("A Go2 WebRTC to udp broadcaster.")
        .setting(AppSettings::DeriveDisplayOrder)
        .subcommand_negates_reqs(true)
        .arg(
            Arg::new("FULLHELP")
                .help("Prints more detailed help information")
                .long("fullhelp"),
        )
        .arg(
            Arg::new("video_port")
                .takes_value(true)
                .short('v')
                .long("video")
                .value_parser(clap::value_parser!(u16))
                .default_value("4002")
                .help("UDP port for video streaming (default 4002)."),
        )
        .arg(
            Arg::new("audio_port")
                .takes_value(true)
                .short('a')
                .long("audio")
                .value_parser(clap::value_parser!(u16)) // Ensure the value is parsed as u16
                .default_value("4000") // Set the default value
                .help("UDP port for audio streaming (default 4000)."),
        )
        .arg(
            Arg::new("robot_ip")
                .takes_value(true)
                .short('r')
                .long("robot")
                .help("IP address of your GO2 robot.")
                .default_value(&robot_ip_env)
        )
        .arg(
            Arg::new("robot_token")
                .takes_value(true)
                .short('t')
                .long("token")
                .default_value(&robot_token_default)
                .help("Authentication token for your GO2 robot."),
        )
        .arg(
            Arg::new("debug")
                .long("debug")
                .short('d')
                .help("Prints debug log information"),
        );

    let matches = app.clone().get_matches();

    if matches.is_present("FULLHELP") {
        app.print_long_help().unwrap();
        std::process::exit(0);
    }

    let video_port: u16 = *matches.get_one("video_port").expect("default not provided");
    let audio_port: u16 = *matches.get_one("audio_port").expect("default not provided");

    let robot_ip: &str = matches
        .get_one::<String>("robot_ip")
        .expect("required unless FULLHELP is present");
    let robot_token: &str = matches
        .get_one::<String>("robot_token")
        .expect("default not provided");

    let mut udp_conns = HashMap::new();
    udp_conns.insert(
        "audio".to_owned(),
        UdpConn {
            conn: {
                let sock = UdpSocket::bind("127.0.0.1:0").await?;
                sock.connect(format!("127.0.0.1:{}", audio_port)).await?;
                Arc::new(sock)
            },
            payload_type: 111,
        },
    );
    udp_conns.insert(
        "video".to_owned(),
        UdpConn {
            conn: {
                let sock = UdpSocket::bind("127.0.0.1:0").await?;
                sock.connect(format!("127.0.0.1:{}", video_port)).await?;
                Arc::new(sock)
            },
            payload_type: 102,
        },
    );

    let debug = matches.is_present("debug");
    if debug {
        env_logger::Builder::new()
            .format(|buf, record| {
                writeln!(
                    buf,
                    "{}:{} [{}] {} - {}",
                    record.file().unwrap_or("unknown"),
                    record.line().unwrap_or(0),
                    record.level(),
                    chrono::Local::now().format("%H:%M:%S.%6f"),
                    record.args()
                )
            })
            .filter(None, log::LevelFilter::Trace)
            .init();
    }

    // Create a MediaEngine object to configure the supported codec
    let mut m = MediaEngine::default();

    m.register_codec(
        RTCRtpCodecParameters {
            capability: RTCRtpCodecCapability {
                mime_type: MIME_TYPE_H264.to_owned(),
                clock_rate: 90000,
                channels: 0,
                sdp_fmtp_line: "".to_owned(),
                rtcp_feedback: vec![],
            },
            payload_type: 102,
            ..Default::default()
        },
        RTPCodecType::Video,
    )?;
    m.register_codec(
        RTCRtpCodecParameters {
            capability: RTCRtpCodecCapability {
                mime_type: MIME_TYPE_OPUS.to_owned(),
                clock_rate: 48000,
                channels: 2,
                sdp_fmtp_line: "".to_owned(),
                rtcp_feedback: vec![],
            },
            payload_type: 111,
            ..Default::default()
        },
        RTPCodecType::Audio,
    )?;

    // Register default codecs
    m.register_default_codecs()?;

    // Create a InterceptorRegistry. This is the user configurable RTP/RTCP Pipeline.
    // This provides NACKs, RTCP Reports and other features. If you use `webrtc.NewPeerConnection`
    // this is enabled by default. If you are manually managing You eUST create a InterceptorRegistry
    // for each PeerConnection.
    let mut registry = Registry::new();

    // Use the default set of Interceptors
    registry = register_default_interceptors(registry, &mut m)?;

    // Create the API object with the MediaEngine
    let api = APIBuilder::new()
        .with_media_engine(m)
        .with_interceptor_registry(registry)
        .build();

    // Prepare the configuration
    let config = RTCConfiguration {
        ice_servers: vec![RTCIceServer {
            // add stun server for non-local use
            //  urls: vec!["stun:stun.l.google.com:19302".to_owned()],
            ..Default::default()
        }],
        ..Default::default()
    };

    // Create a new RTCPeerConnection
    let peer_connection = Arc::new(api.new_peer_connection(config).await?);

    // Create a datachannel with label 'data'
    let data_channel = peer_connection.create_data_channel("data", None).await?;

    peer_connection
        .add_transceiver_from_kind(RTPCodecType::Audio, None)
        .await?;
    peer_connection
        .add_transceiver_from_kind(
            RTPCodecType::Video,
            Some(RTCRtpTransceiverInit {
                direction: RTCRtpTransceiverDirection::Recvonly,
                send_encodings: vec![],
            }),
        )
        .await?;

    // Set a handler for when a new remote track starts, this handler saves buffers to disk as
    // an ivf file, since we could have multiple video tracks we provide a counter.
    // In your application this is where you would handle/process video
    let pc = Arc::downgrade(&peer_connection);
    peer_connection.on_track(Box::new(move |track, _, _| {
        // Send a PLI on an interval so that the publisher is pushing a keyframe every rtcpPLIInterval
        println!("Track has started, of type: {}", track.kind());

        let c = if let Some(c) = udp_conns.get(&track.kind().to_string()) {
            c.clone()
        } else {
            return Box::pin(async {});
        };

        // Send a PLI on an interval so that the publisher is pushing a keyframe every rtcpPLIInterval
        let media_ssrc = track.ssrc();
        let pc2 = pc.clone();
        tokio::spawn(async move {
            let mut result = Result::<usize>::Ok(0);
            while result.is_ok() {
                let timeout = tokio::time::sleep(Duration::from_secs(3));
                tokio::pin!(timeout);

                tokio::select! {
                    _ = timeout.as_mut() =>{
                        if let Some(pc) = pc2.upgrade(){
                            result = pc.write_rtcp(&[Box::new(PictureLossIndication{
                                sender_ssrc: 0,
                                media_ssrc,
                            })]).await.map_err(Into::into);
                        }else{
                            break;
                        }
                    }
                };
            }
        });

        let track2 = track.clone();
        tokio::spawn(async move {
            let mut b = vec![0u8; 1500];
            while let Ok((mut rtp_packet, _)) = track2.read(&mut b).await {
                // Update the PayloadType
                rtp_packet.header.payload_type = c.payload_type;
                let n = rtp_packet.marshal_to(&mut b)?;

                // Write
                if let Err(err) = c.conn.send(&b[..n]).await {
                    if err.to_string().contains("Connection refused") {
                        continue;
                    } else {
                        println!("conn send err: {err}");
                        break;
                    }
                }
            }

            Result::<()>::Ok(())
        });

        Box::pin(async move {})
    }));

    let (done_tx, mut done_rx) = tokio::sync::mpsc::channel::<()>(1);

    peer_connection.on_peer_connection_state_change(Box::new(move |s: RTCPeerConnectionState| {
        println!("Peer Connection State has changed: {s}");

        if s == RTCPeerConnectionState::Failed {
            // Wait until PeerConnection has had no network activity for 30 seconds or another failure. It may be reconnected using an ICE Restart.
            // Use webrtc.PeerConnectionStateDisconnected if you are interested in detecting faster timeout.
            // Note that the PeerConnection may come back from PeerConnectionStateDisconnected.
            println!("Peer Connection has gone to failed exiting");
            let _ = done_tx.try_send(());
        }

        Box::pin(async {})
    }));

    // Register channel opening handling
    let d1 = Arc::clone(&data_channel);
    data_channel.on_open(Box::new(move || {
        println!("Data channel '{}'-'{}' open. Random messages will now be sent to any connected DataChannels every 5 seconds", d1.label(), d1.id());

        let _d2 = Arc::clone(&d1);
        Box::pin(async move {
            let result = Result::<usize>::Ok(0);
            while result.is_ok() {
                let timeout = tokio::time::sleep(Duration::from_secs(5));
                tokio::pin!(timeout);

                tokio::select! {
                    _ = timeout.as_mut() =>{
                        // TODO: heart beat
                        // message = json!({...
                        // result = d2.send_text(message).await.map_err(Into::into);
                    }
                };
            }
        })
    }));

    let is_validated = Arc::new(Mutex::new(false));
    let is_validated_clone = Arc::clone(&is_validated);

    // Register text message handling
    let d_label = data_channel.label().to_owned();
    let d3 = Arc::clone(&data_channel);
    data_channel.on_message(Box::new(move |msg: DataChannelMessage| {
        let msg_str = String::from_utf8(msg.data.to_vec()).unwrap();

        println!("Message from DataChannel '{d_label}': '{msg_str}'");
        let v: Result<serde_json::Value, serde_json::Error> = serde_json::from_str(&msg_str);

        match v {
            Ok(json) => {
                let d4 = Arc::clone(&d3);
                let is_validated_clone_inner = Arc::clone(&is_validated_clone);

                let _ = tokio::spawn(async move {
                    let mut is_validated_lock = is_validated_clone_inner.lock().await;

                    println!("Received json: {json}");
                    if !*is_validated_lock && json["type"] == "validation" {
                        if json["data"] == "Validation Ok." {
                            *is_validated_lock = true;
                            let _ = d4.send_text(r#"{"type":"vid", "data":"on"}"#).await;
                        } else {
                            let message = json!({
                                "type": "validation",
                                "data": encrypt_key(&json["data"].to_string()),
                            });
                            let message_str = message.to_string();

                            println!("Sending '{}' as validation", message_str);

                            let _ = d4.send_text(message_str).await;
                        }
                    }
                });
            }
            Err(e) => {
                println!("Error occurred: {e}");
            }
        }

        Box::pin(async {})
    }));

    // Create offer to send to the robot
    let offer = peer_connection.create_offer(None).await?;

    // Create channel that is blocked until ICE Gathering is complete
    let mut gather_complete = peer_connection.gathering_complete_promise().await;

    // Sets the LocalDescription, and starts our UDP listeners
    peer_connection.set_local_description(offer).await?;

    let _ = gather_complete.recv().await;

    let desc_data: String;

    // Send the offer to the robot via HTTP
    if let Some(local_desc) = peer_connection.local_description().await {
        let json_str = serde_json::to_string(&local_desc)?;
        println!("{json_str}");

        desc_data = match post_offer(&robot_ip, &robot_token, &local_desc.sdp).await {
            Ok(Some(json)) => {
                let line = json.to_string();
                println!("Received response: {}", line);
                line
            }
            Ok(None) => return Err(anyhow!("No response received.")),
            Err(e) => return Err(e.into()),
        }
    } else {
        return Err(anyhow!("generate local_description failed!")); 
    }

    let answer = serde_json::from_str::<RTCSessionDescription>(&desc_data)?;

    // Apply the answer as the remote description
    peer_connection.set_remote_description(answer).await?;

    println!("Press ctrl-c to stop");
    tokio::select! {
        _ = done_rx.recv() => {
            println!("received done signal!");
        }
        _ = tokio::signal::ctrl_c() => {
            println!();
        }
    };

    peer_connection.close().await?;

    Ok(())
}
