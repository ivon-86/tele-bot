# control_robot_pigpio.py - альтернативный модуль с pigpio
"""
Использование pigpio для более стабильного software PWM
Требует установки: sudo apt-get install pigpio python3-pigpio
"""

import pigpio
import time

class ControlServoCam:
    """
    Более стабильное управление сервой через pigpio
    pigpio использует DMA и не зависит от прерываний
    """
    
    def __init__(self, servo_pin=18, min_angle=0, max_angle=180, 
                 default_angle=90, min_pulse=600, max_pulse=2400):
        """
        Инициализация с pigpio
        
        Args:
            min_pulse (int): Минимальная ширина импульса в микросекундах (1000 = 1ms)
            max_pulse (int): Максимальная ширина импульса (2000 = 2ms)
        """
        self.servo_pin = servo_pin
        self.min_angle = min_angle
        self.max_angle = max_angle
        self.min_pulse = min_pulse
        self.max_pulse = max_pulse
        self.current_angle = default_angle
        
        # Подключаемся к pigpio демону
        self.pi = pigpio.pi()
        
        if not self.pi.connected:
            raise RuntimeError("Cannot connect to pigpio daemon. Run: sudo pigpiod")
        
        # Устанавливаем частоту 50 Гц
        self.pi.set_PWM_frequency(servo_pin, 50)
        
        # Устанавливаем начальный угол
        self.set_angle(default_angle)
        
        print(f"Servo (pigpio) initialized on GPIO {servo_pin}")
    
    def _angle_to_pulsewidth(self, angle):
        """Преобразование угла в ширину импульса в микросекундах"""
        angle = max(self.min_angle, min(angle, self.max_angle))
        pulse_width = self.min_pulse + (angle / 180.0) * (self.max_pulse - self.min_pulse)
        return int(pulse_width)
    
    def set_angle(self, angle):
        """Установка угла через pigpio"""
        try:
            if angle < self.min_angle or angle > self.max_angle:
                print(f"Warning: Angle {angle}° out of range")
                angle = max(self.min_angle, min(angle, self.max_angle))
            
            pulse_width = self._angle_to_pulsewidth(angle)
            
            # pigpio позволяет напрямую задать ширину импульса
            self.pi.set_servo_pulsewidth(self.servo_pin, pulse_width)

            # Задержка зависит от разницы углов
            angle_diff = abs(angle - self.current_angle)
            move_time = max(0.05, angle_diff / 180.0 * 0.5)  # 0.5сек на полный оборот
            time.sleep(move_time)

            self.current_angle = angle
            
            # Небольшая задержка для движения
            #time.sleep(0.05)



            
            print(f"Servo (pigpio) angle set to: {angle}° (pulse: {pulse_width}µs)")
            return True
            
        except Exception as e:
            print(f"Error setting servo angle: {e}")
            return False
    
    def cleanup(self):
        """Очистка ресурсов"""
        # Отключаем серву
        self.pi.set_servo_pulsewidth(self.servo_pin, 0)
        # Отключаемся от демона
        self.pi.stop()
        print("Servo (pigpio) resources cleaned up")