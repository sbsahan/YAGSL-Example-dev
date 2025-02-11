# 2025 Başlangıç Kodu

### Sürücü + Encoder
deneme sırası:
1. TalonSRX + CANcoder
2. SparkMAX + CANcoder
3. TalonSRX + SRX Mag
4. SparkMAX + SRX Mag (yapmayalım)

### Yapılacaklar
- [ ] encoder ve 775 farklı gear ratiolara sahip, phoenix tuner üzerinden cancoderın değerine bakılıp ona göre resolution ayarlanıcak (`Constants.STEER_ENCODER_RESOLUTION`) (olmazsa farklı bi çözüm bakılır)
- [ ] encoder offsetine bakılması lazım, düz ayarlandıktan sonra phoenix üzerinden değerlere bakılıp json dosyalarındaki yerlere yazılcak.
- [ ] kontroller iyi değil onların değişmesi lazım
