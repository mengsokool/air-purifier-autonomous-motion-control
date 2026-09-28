# การต่อสาย ESP32-S3 N16R8 แบบ DevKitC-1

ผังนี้สำหรับบอร์ด ESP32-S3-WROOM-1 N16R8 ที่ compatible กับ DevKitC-1
ตรงกับ `include/Config.h` และใช้จอ LCD 20x4 I2C ได้โดยไม่ต้องมี MCP23017

| GPIO | อุปกรณ์ | หน้าที่ |
|---|---|---|
| 18 | PMS3003 TX | UART1 RX; ต้องลดเป็นลอจิก 3.3 V |
| 17 | LCD SDA | ผ่าน BSS138 ฝั่ง 3.3 V |
| 10 | LCD SCL | ผ่าน BSS138 ฝั่ง 3.3 V |
| 8, 9 | INA219 SDA, SCL | บัส I2C1 แยกจากจอ ใช้ลอจิก 3.3 V |
| 7 | HC-SR04 Echo | ลด 5 V เป็น 3.3 V ก่อนเข้า GPIO |
| 6 | HC-SR04 Trig | สัญญาณ 3.3 V ออกไปเซ็นเซอร์ |
| 5 | Fan controller PWM | LEDC 25 kHz / 10-bit; ห้ามต่อเข้ามอเตอร์พัดลมโดยตรง |
| 4 | L298N ENA | PWM มอเตอร์ซ้าย; ถอด jumper ENA |
| 11, 12 | L298N IN1, IN2 | ทิศมอเตอร์ซ้าย |
| 13 | SG90 signal | ใช้ไฟ servo แยกจากบอร์ด |
| 14 | L298N ENB | PWM มอเตอร์ขวา; ถอด jumper ENB |
| 15, 16 | L298N IN3, IN4 | ทิศมอเตอร์ขวา |
| 1 | `S` ของ voltage sensor module | โมดูล 0–25V: `+` ไป battery+, `-` ไป battery-/GND, output 1:5 |
| 2, 21 | Encoder ซ้าย A, B | อินพุต 3.3 V, PCNT hardware x4 decoding |
| 38, 39 | Encoder ขวา A, B | อินพุต 3.3 V, PCNT hardware x4 decoding |

## LCD 20x4 I2C

ต่อ `GPIO17/GPIO10` เข้าด้าน LV ของ BSS138 (`LV=3.3V`) แล้วต่อด้าน HV ไป
ยัง `SDA/SCL` ของ LCD backpack (`HV=5V`). จ่าย LCD ที่ 5V และต่อ GND ร่วมกัน
ไลบรารีค้นหาที่อยู่ I2C และชนิด backpack ให้อัตโนมัติ; ปกติจะเป็น `0x27` หรือ
`0x3F` จึงไม่ต้องแก้โค้ด

## INA219 วัดกระแสชาร์จ

ต่อ `VCC` ของ INA219 กับ 3.3 V, `GND` กับ GND ร่วม, `SDA` กับ GPIO8 และ
`SCL` กับ GPIO9. บัสนี้แยกจากบัสจอและไม่ต้องผ่าน BSS138. ห้ามจ่าย VCC 5 V.
สายกำลังชาร์จให้เดิน `charger + → VIN+`, แล้ว
`VIN- → battery +`; ขั้วลบ charger และ battery ต่อ GND ร่วมกัน. โค้ดใช้
I2C address `0x40` และคาลิเบรต INA219 32 V / 2 A เป็นค่าเริ่มต้น.

เฟิร์มแวร์ใช้กระแสผ่าน INA219 และแรงดันแบตจาก GPIO1 เป็น charge-state
machine สำหรับ SLA 12 V / 12 Ah: `CHARGING → ABSORPTION → FULL`.  สถานะ
FULL จะยืนยันเมื่อแบตอยู่ที่อย่างน้อย 14.3 V และกระแสไม่เกิน 200 mA ต่อเนื่อง
10 นาที; หากเพิ่งมีประวัติ FULL และเสียบใหม่ จะยืนยันซ้ำเพียง 20 วินาทีเมื่อ
เงื่อนไขเดิมยังครบ. ห้ามใช้เกณฑ์นี้กับแบตชนิดอื่นโดยไม่ปรับค่าใน `Config.h`.

## ขาที่ไม่ใช้

- GPIO19/20 สงวนสำหรับ native USB-C
- GPIO0, GPIO3, GPIO45 และ GPIO46 เป็นขา boot/strapping จึงไม่ต่ออุปกรณ์
- GPIO43/44 คือ UART0 จาก USB-to-UART; เว้นไว้สำหรับอัปโหลดและแก้ปัญหา
- GPIO42 คือ `CHARGER_PRESENT`: ต่อจุดกลางของ 180 kOhm (จาก adapter+ หลัง
  ไดโอด) และ 33 kOhm (ลง GND) เข้าขานี้. Adapter 18V จะอ่านเป็น HIGH;
  เฟิร์มแวร์ล็อกคำสั่งมอเตอร์ทันทีเมื่อ HIGH.

## ไฟเลี้ยงและความปลอดภัย

- ต่อ GND ของ ESP32, L298N, PMS3003, encoder, servo, HC-SR04 และ fan driver
  ร่วมกัน
- มอเตอร์, servo และพัดลมต้องใช้แหล่งจ่ายกำลังแยก; ห้ามดึงผ่านขา 5V ของบอร์ด
- ถ้า fan controller ต้องการ PWM 5 V หรือเป็นพัดลม 4-wire ให้ใช้วงจร
  transistor/open-collector ที่ GPIO5; ESP32-S3 ไม่ทนสัญญาณ 5 V
- โมดูลวัดแรงดัน 30 kOhm/7.5 kOhm (1:5) ใช้กับ SLA 12V ได้: 14.4V จะให้
  2.88V ที่ GPIO1. ห้ามวัดเกิน 16.5V เพราะ GPIO ของ ESP32-S3 รับได้ 3.3V
- ทดสอบครั้งแรกโดยยกล้อ และตั้ง `MOTION_ENABLED=false` ก่อน

## Encoder PCNT

ESP32-S3 นับ encoder ด้วย PCNT ในฮาร์ดแวร์ ไม่ได้เรียก GPIO interrupt ทุกขอบ
สัญญาณ จึงคงความละเอียด x4 แต่ลดภาระ CPU และกรอง glitch ที่สั้นกว่า 1 µs ได้
หลังอัปโหลด ให้หมุนล้อไปข้างหน้าทีละล้อแล้วดู serial status: count ของทั้งสองล้อ
ต้องเพิ่มขึ้นขณะรถสั่งเดินหน้า หากล้อใดลดลง ให้สลับค่า
`ENCODER_LEFT_INVERTED` หรือ `ENCODER_RIGHT_INVERTED` ใน `include/Config.h`
ของล้อนั้น
