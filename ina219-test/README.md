# INA219 charger monitor test

โปรแกรมนี้ทดสอบ INA219 เพียงตัวเดียวบนบัส I2C ของรถ:

- `VCC` ไป `3.3V`
- `GND` ไป GND ร่วม
- `SDA` ไป `GPIO8`
- `SCL` ไป `GPIO9`
- A0/A1 ไม่เชื่อม ใช้ address `0x40`

สายกำลังต้องต่ออนุกรมดังนี้:

`XL4015 OUT+ -> INA219 VIN+ -> INA219 VIN- -> battery+`

คำสั่ง build/upload/monitor:

```sh
cd ina219-test
pio run
pio run -t upload
pio device monitor -b 115200
```

ถ้าแสดง `no response from INA219 at 0x40` ให้ตรวจไฟ 3.3V, GND, SDA/SCL
และจุดบัดกรี A0/A1. ถ้าอ่านได้แต่กระแสใกล้ 0 mA ให้ตรวจว่าสายชาร์จกระแสสูง
เดินผ่าน `VIN+` และ `VIN-` จริง. เครื่องหมายกระแสติดลบหมายถึงต่อ VIN+/VIN-
กลับทิศ แต่ยังยืนยันได้ว่าเซนเซอร์กำลังวัดกระแสอยู่.
