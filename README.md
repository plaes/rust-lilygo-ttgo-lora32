# Rust example for the TTGO LoRa32

## Hardware

* ESP32 DOWDQ-V3
* SX1276
* CH9102X - USB to serial UART interface
* 4MB flash
* OLED (SSD1306)
* 1 button (+ reset)
* 1 LED

## Examples in this repository

### LoRa P2P Receiver

```
cargo run --release --bin receive
```

* LoRa P2P Transmit
```
cargo run --release --bin receive
```

![LoRa P2P devices](/img/lora-p2p-transmit.jpg)

## Board pinout

![LILYGO LORA32 V1.0 Pinout](/img/lilygo-ttgo-lora32_v1.0_pinout.jpg)
![LILYGO LORA32 V1.0 Pinout (alterative)](/img/TTGO-LoRa32-OLED-Board-Pinout.jpg)
