#!/usr/bin/env python3
"""
AZAK SİHA - MAVLink Haberleşme Modülü
Jetson Nano <-> Pixhawk arasındaki tüm iletişimi sağlar
"""

import time
import threading
import math
from typing import Optional, Dict, Any, Tuple, Callable

# Ortak tipler
from types import (
    DroneState, Position, Attitude, Velocity, 
    BatteryStatus, GPSStatus, FlightMode
)

try:
    from pymavlink import mavutil
    from pymavlink.dialects.v20 import common as mavlink
except ImportError:
    print("❌ PyMAVLink kurulu değil! Kurulum: pip install pymavlink")
    exit(1)

class MAVLinkHandler:
    """MAVLink haberleşme sınıfı"""
    
    def __init__(self, connection_string: str = "/dev/ttyACM0", baudrate: int = 57600):
        """
        MAVLink bağlantısını başlat
        
        Args:
            connection_string: Bağlantı adresi (/dev/ttyACM0, /dev/ttyUSB0, tcp:127.0.0.1:5760)
            baudrate: Baud rate (genelde 57600 veya 115200)
        """
        self.connection_string = connection_string
        self.baudrate = baudrate
        self.connection = None
        self.drone_state = DroneState()
        
        # Threading kontrolü
        self.running = False
        self.telemetry_thread = None
        
        # Callback fonksiyonları
        self.callbacks = {
            'heartbeat': [],
            'attitude': [],
            'global_position': [],
            'local_position': [],
            'gps': [],
            'battery': [],
            'mode_change': [],
            'arm_disarm': [],
            'mission_item_reached': [],
            'command_ack': []
        }
        
        # Son komut durumları
        self.last_command_ack = None
        self.command_responses = {}
        
        print(f"🔗 MAVLink Handler oluşturuldu: {connection_string}")
    
    def connect(self) -> bool:
        """Pixhawk'a bağlan"""
        try:
            print(f"🔗 Bağlantı kuruluyor: {self.connection_string}")
            
            # MAVLink bağlantısı oluştur
            self.connection = mavutil.mavlink_connection(
                self.connection_string,
                baud=self.baudrate,
                source_system=255,  # Ground Control System ID
                source_component=0
            )
            
            # İlk heartbeat'i bekle
            print("⏳ İlk heartbeat bekleniyor...")
            self.connection.wait_heartbeat(timeout=10)
            
            # Sistem bilgilerini al
            target_system = self.connection.target_system
            target_component = self.connection.target_component
            
            print(f"✅ Bağlantı başarılı!")
            print(f"   Hedef sistem: {target_system}")
            print(f"   Hedef bileşen: {target_component}")
            
            # Telemetri thread'ini başlat
            self.start_telemetry()
            
            # Veri akışını başlat
            self.request_data_streams()
            
            return True
            
        except Exception as e:
            print(f"❌ Bağlantı hatası: {e}")
            return False
    
    def disconnect(self):
        """Bağlantıyı kapat"""
        print("🔌 Bağlantı kapatılıyor...")
        self.running = False
        
        if self.telemetry_thread:
            self.telemetry_thread.join(timeout=2)
        
        if self.connection:
            self.connection.close()
            self.connection = None
        
        print("✅ Bağlantı kapatıldı")
    
    def start_telemetry(self):
        """Telemetri thread'ini başlat"""
        if not self.running:
            self.running = True
            self.telemetry_thread = threading.Thread(target=self._telemetry_loop)
            self.telemetry_thread.daemon = True
            self.telemetry_thread.start()
            print("📡 Telemetri thread'i başlatıldı")
    
    def stop_telemetry(self):
        """Telemetri thread'ini durdur"""
        self.running = False
        if self.telemetry_thread:
            self.telemetry_thread.join(timeout=2)
        print("⏹️ Telemetri thread'i durduruldu")
    
    def _telemetry_loop(self):
        """Telemetri döngüsü - arka planda çalışır"""
        while self.running and self.connection:
            try:
                # MAVLink mesajlarını oku
                msg = self.connection.recv_match(blocking=False, timeout=0.1)
                
                if msg:
                    self._process_message(msg)
                
                time.sleep(0.01)  # 100Hz döngü
                
            except Exception as e:
                print(f"❌ Telemetri döngüsü hatası: {e}")
                time.sleep(0.1)
    
    def _process_message(self, msg):
        """Gelen MAVLink mesajını işle"""
        msg_type = msg.get_type()
        
        try:
            if msg_type == 'HEARTBEAT':
                self._handle_heartbeat(msg)
            elif msg_type == 'ATTITUDE':
                self._handle_attitude(msg)
            elif msg_type == 'GLOBAL_POSITION_INT':
                self._handle_global_position(msg)
            elif msg_type == 'LOCAL_POSITION_NED':
                self._handle_local_position(msg)
            elif msg_type == 'GPS_RAW_INT':
                self._handle_gps(msg)
            elif msg_type == 'SYS_STATUS':
                self._handle_sys_status(msg)
            elif msg_type == 'BATTERY_STATUS':
                self._handle_battery(msg)
            elif msg_type == 'VFR_HUD':
                self._handle_vfr_hud(msg)
            elif msg_type == 'COMMAND_ACK':
                self._handle_command_ack(msg)
            elif msg_type == 'MISSION_ITEM_REACHED':
                self._handle_mission_item_reached(msg)
                
        except Exception as e:
            print(f"❌ Mesaj işleme hatası ({msg_type}): {e}")
    
    def _handle_heartbeat(self, msg):
        """Heartbeat mesajını işle"""
        self.drone_state.heartbeat_time = time.time()
        self.drone_state.flight_mode_num = msg.custom_mode
        self.drone_state.system_status = mavutil.mode_string_v10(msg)
        
        # Armed durumu kontrol et
        armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        
        if armed != self.drone_state.armed:
            self.drone_state.armed = armed
            self._trigger_callbacks('arm_disarm', armed)
        
        # Flight mode değişimi kontrol et
        mode_name = self.get_flight_mode_name(msg.custom_mode)
        if mode_name != self.drone_state.flight_mode:
            old_mode = self.drone_state.flight_mode
            self.drone_state.flight_mode = mode_name
            self._trigger_callbacks('mode_change', {'old': old_mode, 'new': mode_name})
        
        self._trigger_callbacks('heartbeat', msg)
    
    def _handle_attitude(self, msg):
        """Attitude mesajını işle"""
        self.drone_state.attitude.roll = msg.roll
        self.drone_state.attitude.pitch = msg.pitch
        self.drone_state.attitude.yaw = msg.yaw
        self._trigger_callbacks('attitude', msg)
    
    def _handle_global_position(self, msg):
        """Global position mesajını işle"""
        self.drone_state.position.latitude = msg.lat / 1e7
        self.drone_state.position.longitude = msg.lon / 1e7
        self.drone_state.position.altitude = msg.alt / 1000.0
        self.drone_state.position.relative_altitude = msg.relative_alt / 1000.0
        self.drone_state.velocity.x = msg.vx / 100.0
        self.drone_state.velocity.y = msg.vy / 100.0
        self.drone_state.velocity.z = msg.vz / 100.0
        self._trigger_callbacks('global_position', msg)
    
    def _handle_local_position(self, msg):
        """Local position mesajını işle"""
        self._trigger_callbacks('local_position', msg)
    
    def _handle_gps(self, msg):
        """GPS mesajını işle"""
        self.drone_state.gps.fix_type = msg.fix_type
        self.drone_state.gps.satellites_visible = msg.satellites_visible
        self._trigger_callbacks('gps', msg)
    
    def _handle_sys_status(self, msg):
        """System status mesajını işle"""
        self.drone_state.battery.voltage = msg.voltage_battery / 1000.0
        self.drone_state.battery.current = msg.current_battery / 100.0
        self.drone_state.battery.remaining = msg.battery_remaining
    
    def _handle_battery(self, msg):
        """Battery status mesajını işle"""
        self._trigger_callbacks('battery', msg)
    
    def _handle_vfr_hud(self, msg):
        """VFR HUD mesajını işle"""
        self.drone_state.velocity.air_speed = msg.airspeed
        self.drone_state.velocity.ground_speed = msg.groundspeed
    
    def _handle_command_ack(self, msg):
        """Command ACK mesajını işle"""
        self.last_command_ack = msg
        command_id = msg.command
        result = msg.result
        
        self.command_responses[command_id] = {
            'result': result,
            'time': time.time(),
            'success': result == mavutil.mavlink.MAV_RESULT_ACCEPTED
        }
        
        self._trigger_callbacks('command_ack', msg)
    
    def _handle_mission_item_reached(self, msg):
        """Mission item reached mesajını işle"""
        self._trigger_callbacks('mission_item_reached', msg)
    
    def _trigger_callbacks(self, event_type: str, data):
        """Callback fonksiyonlarını tetikle"""
        if event_type in self.callbacks:
            for callback in self.callbacks[event_type]:
                try:
                    callback(data)
                except Exception as e:
                    print(f"❌ Callback hatası ({event_type}): {e}")
    
    def add_callback(self, event_type: str, callback: Callable):
        """Callback fonksiyonu ekle"""
        if event_type in self.callbacks:
            self.callbacks[event_type].append(callback)
        else:
            print(f"⚠️ Bilinmeyen event type: {event_type}")
    
    def request_data_streams(self):
        """Veri akışlarını başlat"""
        if not self.connection:
            return False
        
        try:
            # Position data
            self.connection.mav.request_data_stream_send(
                self.connection.target_system,
                self.connection.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_POSITION,
                4,  # 4Hz
                1   # Start
            )
            
            # Attitude data
            self.connection.mav.request_data_stream_send(
                self.connection.target_system,
                self.connection.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,
                4,  # 4Hz
                1   # Start
            )
            
            # System status
            self.connection.mav.request_data_stream_send(
                self.connection.target_system,
                self.connection.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS,
                2,  # 2Hz
                1   # Start
            )
            
            print("📊 Veri akışları başlatıldı")
            return True
            
        except Exception as e:
            print(f"❌ Veri akışı başlatma hatası: {e}")
            return False
    
    # ===== KOMUT FONKSİYONLARI =====
    
    def arm(self) -> bool:
        """İHA'yı arm et"""
        return self.send_command_long(
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            1,  # Arm
            0, 0, 0, 0, 0, 0
        )
    
    def disarm(self) -> bool:
        """İHA'yı disarm et"""
        return self.send_command_long(
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,  # Disarm
            0, 0, 0, 0, 0, 0
        )
    
    def takeoff(self, altitude: float) -> bool:
        """Takeoff komutu gönder"""
        return self.send_command_long(
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, altitude
        )
    
    def land(self) -> bool:
        """Land komutu gönder"""
        return self.send_command_long(
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0, 0, 0, 0, 0, 0, 0
        )
    
    def rtl(self) -> bool:
        """Return to Launch"""
        return self.send_command_long(
            mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
            0, 0, 0, 0, 0, 0, 0
        )
    
    def set_flight_mode(self, mode: str) -> bool:
        """Uçuş modunu değiştir"""
        try:
            mode_mapping = self.connection.mode_mapping()
            if mode not in mode_mapping:
                print(f"❌ Bilinmeyen uçuş modu: {mode}")
                return False
            
            mode_id = mode_mapping[mode]
            
            self.connection.mav.set_mode_send(
                self.connection.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_id
            )
            
            print(f"✈️ Uçuş modu değiştirildi: {mode}")
            return True
            
        except Exception as e:
            print(f"❌ Uçuş modu değiştirme hatası: {e}")
            return False
    
    def goto_position(self, lat: float, lon: float, alt: float) -> bool:
        """Belirtilen koordinatlara git"""
        try:
            self.connection.mav.set_position_target_global_int_send(
                0,  # time_boot_ms
                self.connection.target_system,
                self.connection.target_component,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                0b0000111111111000,  # type_mask
                int(lat * 1e7),      # lat_int
                int(lon * 1e7),      # lon_int
                alt,                 # alt
                0, 0, 0,            # velocity
                0, 0, 0,            # acceleration
                0, 0                # yaw, yaw_rate
            )
            
            print(f"🎯 Hedef koordinat ayarlandı: {lat:.6f}, {lon:.6f}, {alt}m")
            return True
            
        except Exception as e:
            print(f"❌ Koordinat gönderme hatası: {e}")
            return False
    
    def set_velocity(self, vx: float, vy: float, vz: float, yaw_rate: float = 0) -> bool:
        """Hız kontrolü"""
        try:
            self.connection.mav.set_position_target_local_ned_send(
                0,  # time_boot_ms
                self.connection.target_system,
                self.connection.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                0b0000111111000111,  # type_mask (velocity only)
                0, 0, 0,            # position
                vx, vy, vz,         # velocity
                0, 0, 0,            # acceleration
                0, yaw_rate         # yaw, yaw_rate
            )
            
            return True
            
        except Exception as e:
            print(f"❌ Hız kontrolü hatası: {e}")
            return False
    
    def send_command_long(self, command: int, param1=0, param2=0, param3=0, 
                         param4=0, param5=0, param6=0, param7=0) -> bool:
        """Uzun komut gönder"""
        try:
            self.connection.mav.command_long_send(
                self.connection.target_system,
                self.connection.target_component,
                command,
                0,  # confirmation
                param1, param2, param3, param4, param5, param6, param7
            )
            
            return True
            
        except Exception as e:
            print(f"❌ Komut gönderme hatası: {e}")
            return False
    
    def wait_command_ack(self, command_id: int, timeout: float = 5.0) -> bool:
        """Komut onayını bekle"""
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            if command_id in self.command_responses:
                response = self.command_responses[command_id]
                del self.command_responses[command_id]  # Cleanup
                return response['success']
            
            time.sleep(0.1)
        
        print(f"⏰ Komut onayı timeout: {command_id}")
        return False
    
    # ===== YARDIMCI FONKSİYONLAR =====
    
    def get_flight_mode_name(self, mode_id: int) -> str:
        """Mode ID'den mode adını al"""
        try:
            mode_mapping = self.connection.mode_mapping()
            for name, id_val in mode_mapping.items():
                if id_val == mode_id:
                    return name
            return f"UNKNOWN_{mode_id}"
        except:
            return "UNKNOWN"
    
    def is_connected(self) -> bool:
        """Bağlantı durumunu kontrol et"""
        if not self.connection:
            return False
        
        # Son heartbeat 3 saniye içinde mi?
        return (time.time() - self.drone_state.heartbeat_time) < 3.0
    
    def get_distance_to(self, lat: float, lon: float) -> float:
        """Belirtilen koordinata mesafe hesapla"""
        target_pos = Position(latitude=lat, longitude=lon)
        return self.drone_state.position.distance_to(target_pos)
    
    def get_bearing_to(self, lat: float, lon: float) -> float:
        """Belirtilen koordinata bearing hesapla"""
        target_pos = Position(latitude=lat, longitude=lon)
        return self.drone_state.position.bearing_to(target_pos)
    
    def get_status_text(self) -> str:
        """Sistem durumu metni"""
        return self.drone_state.get_status_summary()

# ===== TEST FONKSİYONU =====

def test_mavlink():
    """MAVLink test fonksiyonu"""
    print("🧪 MAVLink Test Başlatılıyor...")
    
    # Handler oluştur
    handler = MAVLinkHandler("/dev/ttyACM0")  # Gerçek port
    # handler = MAVLinkHandler("tcp:127.0.0.1:5760")  # SITL için
    
    # Callback fonksiyonları ekle
    def on_heartbeat(msg):
        print(f"💓 Heartbeat alındı - Mode: {handler.drone_state.flight_mode}")
    
    def on_attitude(msg):
        print(f"🎯 Attitude - R:{handler.drone_state.attitude.roll_degrees():.1f}° "
              f"P:{handler.drone_state.attitude.pitch_degrees():.1f}° "
              f"Y:{handler.drone_state.attitude.yaw_degrees():.1f}°")
    
    def on_gps(msg):
        print(f"🛰️ GPS - {handler.drone_state.gps.quality_text()}, "
              f"Satellites: {handler.drone_state.gps.satellites_visible}")
    
    handler.add_callback('heartbeat', on_heartbeat)
    handler.add_callback('attitude', on_attitude)
    handler.add_callback('gps', on_gps)
    
    try:
        # Bağlan
        if not handler.connect():
            print("❌ Bağlantı başarısız!")
            return
        
        print("\n📊 Telemetri verilerini izlemeye başlıyor...")
        print("Çıkmak için Ctrl+C basın\n")
        
        # Ana döngü
        while True:
            time.sleep(1)
            print(f"\n{handler.get_status_text()}\n")
    
    except KeyboardInterrupt:
        print("\n🛑 Test durduruldu")
    
    finally:
        handler.disconnect()

if __name__ == "__main__":
    test_mavlink()
