Amir Gorkovchenko
v2.0.0
changes:
  - implement method to check heartbeats if module is alive and working
    If module stops sending heartbeats, "TuyaWifi.mcu_get_wifi_work_state()" will equal "WIFI_STATE_UNKNOWN"
    HeartBeat is expected around every 15s
  - change "WIFI_SATE_UNKNOW" to "WIFI_STATE_UNKNOWN"
