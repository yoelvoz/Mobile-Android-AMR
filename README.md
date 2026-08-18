# Mobile-Android-AMR

Aplikasi Android untuk mengontrol **AMR (Autonomous Mobile Robot)**, terintegrasi dengan mikrokontroler **ESP32** dan **Arduino Mega**.

## 📱 Tentang Project

Project ini merupakan sistem kontrol mobile robot yang terdiri dari:
- **Aplikasi Android** — antarmuka untuk mengontrol dan memonitor robot
- **ESP32** — modul komunikasi/sensor pada robot
- **Arduino Mega** — pengendali utama pergerakan robot

## 🎥 Implementasi Hardware AMR

https://github.com/user-attachments/assets/9d3f0f4b-db73-4f1d-b345-94d996787842

## 📂 Struktur Repository

| Folder/File | Keterangan |
|---|---|
| `Android Studio/` | Source code aplikasi Android |
| `ESP32/` | Source code firmware ESP32 |
| `MEGA/` | Source code firmware Arduino Mega |
| `Dokumentasi Pengujian/` | Dokumentasi hasil pengujian sistem |
| `Hasil Pengujian/` | Data/hasil pengujian |
| `app-debug-download.apk` | File APK aplikasi Android (versi debug) siap install |

## 🚀 Cara Menggunakan

1. **Aplikasi Android**
   - Download `app-debug-download.apk` lalu install di perangkat Android, **atau**
   - Buka folder `Android Studio/` dengan Android Studio untuk build sendiri

2. **Firmware ESP32 & Arduino Mega**
   - Buka folder `ESP32/` dan `MEGA/` menggunakan Arduino IDE
   - Upload firmware ke masing-masing board

3. **Hubungkan sistem**
   - Pastikan ESP32 dan Arduino Mega terhubung sesuai wiring pada robot
   - Buka aplikasi Android dan hubungkan ke robot

## 🛠️ Teknologi

- Android (Java/Kotlin)
- ESP32
- Arduino Mega

## 📄 Dokumentasi

Dokumentasi pengujian dan hasilnya tersedia di folder [`Dokumentasi Pengujian`](./Dokumentasi%20Pengujian) dan [`Hasil Pengujian`](./Hasil%20Pengujian).

## 👤 Author

[yoelvoz](https://github.com/yoelvoz)
