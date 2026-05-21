### Build and flash
```
west build -b tam_board/nrf5340/cpuapp --sysbuild -- -DCONFIG_AUDIO_DEV=2
```
```
west flash --runner jlink
```


### RTT Logs

In one terminal:
```
JLinkRTTLogger -Device NRF5340_XXAA_APP -If SWD -Speed 4000 -RTTChannel 0 /tmp/rtt_log.txt
```

In another terminal:
```
tail -f /tmp/rtt_log.txt
```



### Troubleshooting

Sometimes the network core is protected and refuses to flash. Try:
```
nrfjprog --recover --family NRF53
```
