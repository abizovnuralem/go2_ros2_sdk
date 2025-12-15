# 1. ZRAM(압축 램) 비활성화 (CPU 부하 줄임 + 실제 스왑 사용)
sudo systemctl disable nvzramconfig
sudo systemctl stop nvzramconfig

# 2. 8GB 스왑 파일 생성 (기존 스왑이 있다면 삭제 후 재생성 권장)
sudo fallocate -l 8G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile

# 3. 부팅 시 자동 적용 (/etc/fstab 파일 끝에 추가)
echo "/swapfile none swap sw 0 0" | sudo tee -a /etc/fstab