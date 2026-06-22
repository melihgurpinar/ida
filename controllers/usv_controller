import math
from controller import Robot, GPS, InertialUnit, Radar, Motor

# 1. Simülasyon ve Cihaz Kurulumları
robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Sensörleri Tanımlama ve Aktif Etme
gps = robot.getDevice("gps")
gps.enable(timestep)

imu = robot.getDevice("imu")
imu.enable(timestep)

radar = robot.getDevice("radar")
radar.enable(timestep)

left_motor = robot.getDevice("left_motor")
right_motor = robot.getDevice("right_motor")
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

# 2. BAŞLANGIÇ SİSTEM PARAMETRELERİ (Sizin Matematiksel Modelleriniz)
# Madde 2: Batarya Değişkenleri
V_bat = 24.0      # V
Ah = 200.0        # Ah
N_bat = 4
E_toplam = (V_bat * Ah * N_bat) / 1000.0 # 19.2 kWh
DoD = 0.8
eta_sistem = 0.9
current_battery_energy = E_toplam # Anlık kalan enerji (kWh)

# Madde 6 & 7: Jeneratör ve Yakıt Değişkenleri
Fuel_tank = 60.0  # Litre
SFC = 0.25        # L/kWh
current_fuel = Fuel_tank

# Durumsal Değişkenler
P_hotel = 2.0     # kW (Radar, PC, Kamera yükü)
hybrid_mode = "SESSIZ_ELEKTRIK" # Başlangıç modu
time_elapsed = 0.0 # Saat cinsinden geçen süre

# Eşik Değerleri (Madde 18)
CPA_safe = 5.0     # Metre
TCPA_limit = 15.0  # Saniye

print(f"--- Hibrit İDA Başlatıldı ---")
print(f"Toplam Batarya Kapasitesi: {E_toplam} kWh | Yakıt Tankı: {Fuel_tank} L")

# Ana Simülasyon Döngüsü
while robot.step(timestep) != -1:
    # Zaman güncellemesi (Simülasyondan gerçek saate dönüşüm)
    dt = timestep / 1000.0  # saniye
    dt_hours = dt / 3600.0  # saat
    time_elapsed += dt_hours
    
    # 3. SENSÖR VERİLERİNİN ALINMASI (Madde 12 & 13)
    gps_vals = gps.getValues() # [x, y, z]
    imu_vals = imu.getRollPitchYaw() # [roll, pitch, yaw]
    
    # Madde 1: Hız ve Konum Hesaplama
    # Basitçe motor hızlarından tahmini bir hız üretelim (Simülasyon hızı knot cinsinden)
    current_speed_knots = 6.0 # Sabit 6 knot seyir varsayımı
    
    # 4. GÜÇ VE ENERJİ TÜKETİM MOTORU (Madde 3, 6 & 7)
    # İleri itki için motorlara güç verelim
    motor_speed = 10.0
    
    if hybrid_mode == "SESSIZ_ELEKTRIK":
        P_propulsion = 4.0 # kW (İtki gücü)
        P_yuk = P_propulsion + P_hotel
        
        # Bataryadan enerji düşümü
        energy_consumed = P_yuk * dt_hours
        current_battery_energy -= (energy_consumed / eta_sistem)
        
        # Madde 3: Kalan Elektrikli Seyir Süresi Hesabı
        T_elektrik_kalan = (max(0.0, current_battery_energy) * DoD * eta_sistem) / P_yuk
        
        # Batarya kritik seviyeye (%20) düşerse Jeneratörü çalıştır (Hibrit geçişi)
        if current_battery_energy <= (E_toplam * (1 - DoD)):
            hybrid_mode = "DIZEL_JENERATOR"
            print(f"\n[UYARI] Batarya kritik seviyede! Dizel Jeneratör Devreye Alındı. Zaman: {time_elapsed*3600:.1f} sn")
            
    elif hybrid_mode == "DIZEL_JENERATOR":
        P_gen = 13.0 # kW jeneratör üretimi (Madde 5)
        P_propulsion = 4.0
        P_charge = 3.0 # Bataryayı arka planda şarj etme gücü
        
        # Madde 6: Yakıt Tüketimi
        fuel_consumed = SFC * P_gen * dt_hours
        current_fuel -= fuel_consumed
        
        # Bataryayı şarj et
        current_battery_energy += P_charge * dt_hours * 0.9 # %90 şarj verimi
        if current_battery_energy > E_toplam:
            current_battery_energy = E_toplam
            
        T_elektrik_kalan = 0.0
        
        if current_fuel <= 0:
            current_fuel = 0
            print("[KRİTİK] Yakıt bitti, sistem duruyor!")
            motor_speed = 0.0

    # 5. OTONOM KAÇINMA VE COLREG MOTORU (Madde 16, 17 & 18)
    # Radar verilerini işleme
    targets = radar.getTargets()
    steering_bias = 0.0 # Dümen yönü sapması
    
    if len(targets) > 0:
        # En yakın hedefi seç
        target = targets[0]
        r_distance = target.distance # Göreli konum (Mesafe)
        r_speed = target.speed       # Göreli hız
        
        # Madde 16 & 17: CPA ve TCPA Basitleştirilmiş Yaklaşım
        if r_speed != 0:
            TCPA = r_distance / abs(r_speed)
        else:
            TCPA = float('inf')
            
        CPA = r_distance # Basit dairesel radar yaklaşımı
        
        # Madde 17: Çarpışma Riski Hesaplama
        if CPA != 0 and TCPA != 0:
            Risk = (1.0 / CPA) * (1.0 / TCPA)
        else:
            Risk = 0
            
        # Madde 18: Otonom Kaçınma Kararı
        if CPA < CPA_safe and TCPA < TCPA_limit:
            # COLREG Kural 14/15 uyarınca Sancağa (Sağa) Kaçış Manevrası yap
            steering_bias = 4.0 
            print(f"[COLREG] Engel Tespit Edildi! CPA: {CPA:.2f}m, TCPA: {TCPA:.1f}sn, Risk: {Risk:.4f} -> Sancağa Manevra yapılıyor.")
    
    # Motor Hızlarını Dümen Sapmasına Göre Uygulama (Diferansiyel Sürüş)
    left_motor.setVelocity(motor_speed + steering_bias)
    right_motor.setVelocity(motor_speed - steering_bias)
    
    # 6. TELEMETRİ EKRANI (Her 2 saniyede bir terminale basar)
    if int(time_elapsed * 3600) % 2 == 0:
        print(f"\n--- Canlı İDA Telemetri Verileri ---")
        print(f"Konum X: {gps_vals[0]:.2f}m | Konum Y: {gps_vals[1]:.2f}m | Mod: {hybrid_mode}")
        print(f"Kalan Batarya: {current_battery_energy:.2f} kWh (%{(current_battery_energy/E_toplam)*100:.1f})")
        print(f"Kalan Yakıt: {current_fuel:.2f} Litre | Tahmini Kalan Sessiz Süre: {T_elektrik_kalan:.2f} saat")
