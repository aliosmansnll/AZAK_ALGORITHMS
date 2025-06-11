#!/usr/bin/env python3
"""
AZAK SİHA - Güçlendirilmiş PID Kontrol Modülü
Yaw, pitch ve roll eksenlerinde otonom yönlendirme için PID kontrolü sağlar.
"""

import time

class PIDController:
    def __init__(self, kp=1.0, ki=0.0, kd=0.0, output_limits=(-1.0, 1.0), integral_limit=100.0):
        """
        PID katsayıları ve limitler tanımlanır.
        - output_limits: PID çıkışının sınırlandırılması (örneğin: servo sinyali -1.0 ile 1.0 arası)
        - integral_limit: integral birikmesinin sınırı (windup koruması)
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limits = output_limits
        self.integral_limit = integral_limit

        # İçsel PID değişkenleri
        self.integral = 0.0
        self.previous_error = 0.0
        self.previous_time = None

    def reset(self):
        """
        PID kontrol değerlerini sıfırlar. Mod değişimlerinde çağrılabilir.
        """
        self.integral = 0.0
        self.previous_error = 0.0
        self.previous_time = None

    def update(self, error, current_time=None):
        """
        Hata değerine göre PID çıktısını hesaplar.
        """
        if current_time is None:
            current_time = time.time()

        if self.previous_time is None:
            self.previous_time = current_time
            return 0.0  # İlk güncellemede kontrol uygulanmaz

        dt = current_time - self.previous_time
        if dt < 1e-6:
            return 0.0  # Zaman çok kısa ise sapmaları engelle

        de = error - self.previous_error

        # --- PID Bileşenleri Hesaplama ---
        p = self.kp * error

        self.integral += error * dt
        # Integral Windup Sınırlandırması
        self.integral = max(min(self.integral, self.integral_limit), -self.integral_limit)
        i = self.ki * self.integral

        d = self.kd * (de / dt)

        # Toplam çıktı
        output = p + i + d

        # Sınırlandırılmış çıktı
        min_out, max_out = self.output_limits
        output = max(min(output, max_out), min_out)

        # Değer güncellemeleri
        self.previous_error = error
        self.previous_time = current_time

        return output
