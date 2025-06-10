#!/usr/bin/env python3
"""
AZAK SİHA - Ortak Veri Tipleri
Tüm modüllerde kullanılacak ortak sınıflar ve enum'lar
"""

import time
import math
from enum import Enum
from dataclasses import dataclass
from typing import Optional, Dict, Any, Tuple, List

class FlightState(Enum):
    """Uçuş durumları - Ana algoritma state machine"""
    IDLE = "idle"
    AUTONOMOUS_TAKEOFF = "otonom_kalkis"
    FREE_FLIGHT = "serbest_ucus"
    HSS_CHECK = "hss_kontrol"
    TRACKING_CHECK = "takip_kontrol" 
    TARGET_SELECTION = "hedef_secim"
    LOCK_ON = "kilitlenme"
    LOCK_TIMER = "kilitlenme_sayac"
    MISSION_COMPLETE = "gorev_tamam"
    EVASION = "kacis"
    LANDING = "inis"

class FlightMode(Enum):
    """Pixhawk uçuş modları"""
    STABILIZE = 0
    ACRO = 1
    ALT_HOLD = 2
    AUTO = 3
    GUIDED = 4
    LOITER = 5
    RTL = 6
    CIRCLE = 7
    LAND = 9
    BRAKE = 17
    AVOID_ADSB = 18
    GUIDED_NOGPS = 20
    SMART_RTL = 21

class MissionType(Enum):
    """Görev tipleri"""
    LOCK_ON = "lock_on"
    KAMIKAZE = "kamikaze"
    EVASION = "evasion"
    FREE_FLIGHT = "free_flight"

@dataclass
class Position:
    """3D konum bilgisi"""
    latitude: float = 0.0
    longitude: float = 0.0
    altitude: float = 0.0
    relative_altitude: float = 0.0
    
    def distance_to(self, other: 'Position') -> float:
        """İki konum arası mesafe (Haversine)"""
        R = 6371000  # Earth radius in meters
        
        lat1_rad = math.radians(self.latitude)
        lat2_rad = math.radians(other.latitude)
        delta_lat = math.radians(other.latitude - self.latitude)
        delta_lon = math.radians(other.longitude - self.longitude)
        
        a = (math.sin(delta_lat/2) * math.sin(delta_lat/2) +
             math.cos(lat1_rad) * math.cos(lat2_rad) *
             math.sin(delta_lon/2) * math.sin(delta_lon/2))
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        
        return R * c
    
    def bearing_to(self, other: 'Position') -> float:
        """İki konum arası bearing"""
        lat1_rad = math.radians(self.latitude)
        lat2_rad = math.radians(other.latitude)
        delta_lon = math.radians(other.longitude - self.longitude)
        
        y = math.sin(delta_lon) * math.cos(lat2_rad)
        x = (math.cos(lat1_rad) * math.sin(lat2_rad) -
             math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(delta_lon))
        
        bearing = math.atan2(y, x)
        return math.degrees(bearing)

@dataclass
class Attitude:
    """Duruş bilgisi (radyan)"""
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    
    def roll_degrees(self) -> float:
        return math.degrees(self.roll)
    
    def pitch_degrees(self) -> float:
        return math.degrees(self.pitch)
    
    def yaw_degrees(self) -> float:
        return math.degrees(self.yaw)

@dataclass
class Velocity:
    """Hız bilgisi (m/s)"""
    x: float = 0.0  # North/Forward
    y: float = 0.0  # East/Right
    z: float = 0.0  # Down
    ground_speed: float = 0.0
    air_speed: float = 0.0
    
    def magnitude(self) -> float:
        """Toplam hız büyüklüğü"""
        return math.sqrt(self.x**2 + self.y**2 + self.z**2)

@dataclass
class BatteryStatus:
    """Batarya durumu"""
    voltage: float = 0.0        # Volt
    current: float = 0.0        # Amper
    remaining: int = 100        # Yüzde
    consumed: int = 0           # mAh
    temperature: float = 0.0    # Celsius
    
    def is_low(self, threshold: int = 20) -> bool:
        """Batarya düşük mü?"""
        return self.remaining < threshold
    
    def is_critical(self, threshold: int = 10) -> bool:
        """Batarya kritik seviyede mi?"""
        return self.remaining < threshold

@dataclass
class GPSStatus:
    """GPS durumu"""
    fix_type: int = 0           # 0: No GPS, 1: No Fix, 2: 2D Fix, 3: 3D Fix
    satellites_visible: int = 0
    hdop: float = 0.0          # Horizontal Dilution of Precision
    vdop: float = 0.0          # Vertical Dilution of Precision
    
    def has_fix(self) -> bool:
        """GPS fix var mı?"""
        return self.fix_type >= 3 and self.satellites_visible >= 6
    
    def quality_text(self) -> str:
        """GPS kalite metni"""
        if self.fix_type == 0:
            return "GPS YOK"
        elif self.fix_type == 1:
            return "FIX YOK"
        elif self.fix_type == 2:
            return "2D FIX"
        elif self.fix_type == 3:
            return "3D FIX"
        else:
            return "UNKNOWN"

@dataclass
class DroneState:
    """İHA'nın tam durumu - Tüm modüllerde kullanılacak"""
    
    # Konum ve hareket bilgileri
    position: Position = None
    attitude: Attitude = None
    velocity: Velocity = None
    
    # Sistem durumu
    armed: bool = False
    flight_mode: str = "UNKNOWN"
    flight_mode_num: int = 0
    system_status: str = "UNKNOWN"
    
    # Sensör durumları
    battery: BatteryStatus = None
    gps: GPSStatus = None
    
    # Çevre verileri
    temperature: float = 0.0    # Celsius
    pressure: float = 0.0       # mBar
    
    # Bağlantı durumu
    heartbeat_time: float = 0.0
    connection_quality: float = 0.0  # 0-100
    
    def __post_init__(self):
        """Varsayılan değerleri ayarla"""
        if self.position is None:
            self.position = Position()
        if self.attitude is None:
            self.attitude = Attitude()
        if self.velocity is None:
            self.velocity = Velocity()
        if self.battery is None:
            self.battery = BatteryStatus()
        if self.gps is None:
            self.gps = GPSStatus()
    
    def is_connected(self, timeout: float = 3.0) -> bool:
        """Bağlantı var mı?"""
        return (time.time() - self.heartbeat_time) < timeout
    
    def is_ready_for_flight(self) -> bool:
        """Uçuşa hazır mı?"""
        return (
            self.is_connected() and
            self.gps.has_fix() and
            not self.battery.is_critical() and
            self.armed
        )
    
    def can_arm(self) -> bool:
        """Arm edilebilir mi?"""
        return (
            self.is_connected() and
            self.gps.has_fix() and
            not self.battery.is_critical() and
            not self.armed
        )
    
    def get_status_summary(self) -> str:
        """Özet durum metni"""
        status = f"🛩️ İHA Durumu:\n"
        status += f"   Bağlantı: {'✅' if self.is_connected() else '❌'}\n"
        status += f"   Armed: {'✅' if self.armed else '❌'}\n"
        status += f"   Mode: {self.flight_mode}\n"
        status += f"   GPS: {self.gps.quality_text()} ({self.gps.satellites_visible} uydu)\n"
        status += f"   Batarya: {self.battery.remaining}% ({self.battery.voltage:.1f}V)\n"
        status += f"   Altitude: {self.position.relative_altitude:.1f}m\n"
        status += f"   Speed: {self.velocity.ground_speed:.1f}m/s\n"
        status += f"   Ready: {'✅' if self.is_ready_for_flight() else '❌'}"
        
        return status

@dataclass
class TargetInfo:
    """Hedef bilgisi (rakip İHA veya QR kod)"""
    id: int
    position: Position
    last_seen: float = 0.0
    confidence: float = 0.0
    target_type: str = "drone"  # "drone", "qr_code"
    
    def is_fresh(self, max_age: float = 2.0) -> bool:
        """Hedef bilgisi güncel mi?"""
        return (time.time() - self.last_seen) < max_age
    
    def distance_to_drone(self, drone_pos: Position) -> float:
        """İHA'ya olan mesafe"""
        return drone_pos.distance_to(self.position)

@dataclass
class HSSZone:
    """Hava Savunma Sistemi yasaklı bölge"""
    id: int
    center: Position
    radius: float  # metre
    active: bool = True
    
    def contains_point(self, point: Position) -> bool:
        """Nokta bu bölgede mi?"""
        if not self.active:
            return False
        
        distance = self.center.distance_to(point)
        return distance <= self.radius
    
    def distance_to_edge(self, point: Position) -> float:
        """Bölge kenarına olan mesafe (negatif = içeride)"""
        distance_to_center = self.center.distance_to(point)
        return distance_to_center - self.radius

@dataclass
class MissionData:
    """Görev verileri"""
    current_mission: MissionType = MissionType.FREE_FLIGHT
    
    # Hedef bilgileri
    target_coordinates: Optional[Position] = None
    current_target: Optional[TargetInfo] = None
    all_targets: Dict[int, TargetInfo] = None
    
    # Kilitlenme bilgileri
    lock_start_time: Optional[float] = None
    lock_duration: float = 0.0
    successful_locks: int = 0
    
    # HSS bilgileri
    hss_zones: List[HSSZone] = None
    
    # Görev zamanlaması
    mission_start_time: float = 0.0
    mission_duration: float = 15 * 60  # 15 dakika
    
    def __post_init__(self):
        """Varsayılan değerleri ayarla"""
        if self.all_targets is None:
            self.all_targets = {}
        if self.hss_zones is None:
            self.hss_zones = []
        if self.mission_start_time == 0.0:
            self.mission_start_time = time.time()
    
    def is_mission_time_up(self) -> bool:
        """Görev süresi doldu mu?"""
        elapsed = time.time() - self.mission_start_time
        return elapsed >= self.mission_duration
    
    def remaining_time(self) -> float:
        """Kalan süre (saniye)"""
        elapsed = time.time() - self.mission_start_time
        return max(0, self.mission_duration - elapsed)
    
    def is_in_any_hss_zone(self, position: Position) -> bool:
        """Herhangi bir HSS bölgesinde mi?"""
        return any(zone.contains_point(position) for zone in self.hss_zones)
    
    def get_nearest_hss_zone(self, position: Position) -> Optional[HSSZone]:
        """En yakın HSS bölgesi"""
        if not self.hss_zones:
            return None
        
        return min(
            self.hss_zones,
            key=lambda zone: zone.center.distance_to(position)
        )
    
    def add_target(self, target: TargetInfo):
        """Hedef ekle"""
        self.all_targets[target.id] = target
    
    def update_target(self, target_id: int, position: Position, confidence: float):
        """Hedef güncelle"""
        if target_id in self.all_targets:
            self.all_targets[target_id].position = position
            self.all_targets[target_id].confidence = confidence
            self.all_targets[target_id].last_seen = time.time()
        else:
            # Yeni hedef oluştur
            new_target = TargetInfo(
                id=target_id,
                position=position,
                confidence=confidence,
                last_seen=time.time()
            )
            self.add_target(new_target)
    
    def get_fresh_targets(self, max_age: float = 5.0) -> List[TargetInfo]:
        """Güncel hedefleri getir"""
        current_time = time.time()
        return [
            target for target in self.all_targets.values()
            if (current_time - target.last_seen) < max_age
        ]

# ===== YARDIMCI FONKSİYONLAR =====

def create_position(lat: float, lon: float, alt: float = 0.0) -> Position:
    """Konum oluşturma helper"""
    return Position(latitude=lat, longitude=lon, altitude=alt)

def create_hss_zone(zone_id: int, lat: float, lon: float, radius: float) -> HSSZone:
    """HSS bölgesi oluşturma helper"""
    center = create_position(lat, lon)
    return HSSZone(id=zone_id, center=center, radius=radius)

def degrees_to_radians(degrees: float) -> float:
    """Derece -> Radyan"""
    return math.radians(degrees)

def radians_to_degrees(radians: float) -> float:
    """Radyan -> Derece"""
    return math.degrees(radians)

def normalize_angle(angle: float) -> float:
    """Açıyı -180 ile +180 arasında normalize et"""
    while angle > 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle

def angle_difference(angle1: float, angle2: float) -> float:
    """İki açı arasındaki fark"""
    diff = angle2 - angle1
    return normalize_angle(diff)