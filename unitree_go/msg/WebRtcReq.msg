# message header id. if 0, it will be assigned automatically
int64  id
# topic name on dog's side including rt/ prefix
string topic
# api_id, see https://wiki.theroboverse.com/en/unitree-go2-app-console-commands#sending-commands-to-go2
int64 api_id
# payload for specific api_id
string parameter
# priority of the message. 0 non-priority, 1 priority
uint8 priority