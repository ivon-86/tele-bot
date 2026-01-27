# control_robot_pigpio.py - исправленная версия
import pigpio
import time
import math

import sys
# # ============================================================================
# # ИНИЦИАЛИЗАЦИЯ PIGPIO
# # ============================================================================

# pi = pigpio.pi()
# if not pi.connected:
#     print("Ошибка: Не удалось подключиться к pigpio демону")
#     print("Запустите: sudo pigpiod")
#     sys.exit(1)
from control_motor import *

def cleanup():
        pi.stop()

class RobotChassis:
    def __init__(self):
        # Создаём моторы
        self.left_motor = Motor(LEFT_PWM_PIN, LEFT_IN1_PIN, LEFT_IN2_PIN, "Левый")
        self.right_motor = Motor(RIGHT_PWM_PIN, RIGHT_IN1_PIN, RIGHT_IN2_PIN, "Правый")
        
        # Создаём счётчики энкодеров
        self.left_encoder = EncoderCounter(LEFT_ENC_A, "Левый энк.")
        self.right_encoder = EncoderCounter(RIGHT_ENC_A, "Правый энк.")
        self.curent_speed = 0

    
    def transform_value_control_speed(self, speed): # Преобразуем входные данные от джойстика -100 : 100 в данные заполнения PWM в соответствии с настройками мотора
        if speed < 0:
            # Исходный диапазон: -100..0
            # Новый диапазон: -MAX_PWM..-MIN_PWM
            return int(((speed - (-MAX_PWM)) / (0 - (-MAX_PWM))) * (-MIN_PWM - (-MAX_PWM)) + (-MAX_PWM))
        else:
            # Исходный диапазон: 0..100
            # Новый диапазон: MIN_PWM..MAX_PWM
            return int((speed / MAX_PWM) * (MAX_PWM - MIN_PWM) + MIN_PWM)
        
    def move_robot(self, controlX, controlY):
        
        controlX //= controlX
        speed_left = self.transform_value_control_speed(max(-MAX_PWM, min(MAX_PWM * (controlY + controlX), MAX_PWM)))    # преобразуем скорость робота,
        speed_right = self.transform_value_control_speed(max(-MAX_PWM, min(MAX_PWM * (controlY - controlX), MAX_PWM)))    # в зависимости от положения джойстика
        print(f'speed_left - {speed_left},\t speed_right - {speed_right}') # для отладки
        self.left_motor.set_pwm_smooth(speed_left)
        self.right_motor.set_pwm_smooth(speed_right)

        # speedA = max(-MAX_PWM, min(speedA, MAX_PWM))    # функция аналогичная constrain в arduino
        # speedB = max(-MAX_PWM, min(speedB, MAX_PWM))    # функция аналогичная constrain в arduino

    def stop_robot(self):
        self.left_motor.stop()
        self.right_motor.stop()

    
    
    def stop(self):
        """Остановка"""
        print("⏹ СТОП")
        self.left_motor.stop()
        self.right_motor.stop()



class ControlServoCam:
    """
    Плавное управление сервой через pigpio
    """
    
    def __init__(self, servo_pin=24, min_angle=0.0, max_angle=180.0, 
                 default_angle=90.0, min_pulse=600, max_pulse=2400,
                 speed_factor=1.0):
        """
        Args:
            speed_factor (float): Коэффициент скорости (0.1 = медленно, 2.0 = быстро)
        """
        self.servo_pin = servo_pin
        self.min_angle = float(min_angle)
        self.max_angle = float(max_angle)
        self.min_pulse = min_pulse
        self.max_pulse = max_pulse
        self.current_angle = float(default_angle)
        self.target_angle = float(default_angle)
        self.speed_factor = speed_factor
        self.is_moving = False
        
        # # Подключаемся к pigpio демону
        # self.pi = pigpio.pi()
        
        # if not self.pi.connected:
        #     raise RuntimeError("Cannot connect to pigpio daemon. Run: sudo pigpiod")
        
        # Устанавливаем начальный угол
        self._set_angle_direct(default_angle)
        
        print(f"Servo (pigpio) initialized on GPIO {servo_pin}")
        print(f"Angle range: {min_angle}° - {max_angle}°, Pulses: {min_pulse}-{max_pulse}µs")
    
    def _angle_to_pulsewidth(self, angle):
        """Преобразование угла в ширину импульса"""
        angle = max(self.min_angle, min(angle, self.max_angle))
        pulse_width = self.min_pulse + (angle / 180.0) * (self.max_pulse - self.min_pulse)
        return int(pulse_width)
    
    def _set_angle_direct(self, angle):
        """Непосредственная установка угла без плавности"""
        try:
            pulse_width = self._angle_to_pulsewidth(angle)
            pi.set_servo_pulsewidth(self.servo_pin, pulse_width)
            self.current_angle = float(angle)
            return True
        except Exception as e:
            print(f"Error in direct angle set: {e}")
            return False
    
    def set_angle(self, angle, smooth=True, duration=None):
        """
        Установка угла с возможностью плавного движения
        
        Args:
            angle (float): Целевой угол (дробный)
            smooth (bool): Плавное движение или мгновенное
            duration (float): Длительность движения в секундах (None = автоматически)
        
        Returns:
            bool: Успех операции
        """
        try:
            # Приводим к float и ограничиваем
            target_angle = float(angle)
            target_angle = max(self.min_angle, min(target_angle, self.max_angle))
            
            # Если угол не изменился
            if abs(target_angle - self.current_angle) < 0.1:
                return True
            
            self.target_angle = target_angle
            
            if smooth:
                # Плавное движение
                return self._move_smoothly(target_angle, duration)
            else:
                # Мгновенное движение
                return self._set_angle_direct(target_angle)
                
        except Exception as e:
            print(f"Error setting angle: {e}")
            return False
    
    def _move_smoothly(self, target_angle, duration=None):
        """Плавное движение к целевому углу с высокой точностью"""
        if self.is_moving:
            print("Servo is already moving")
            return False
        
        self.is_moving = True
        
        try:
            start_angle = self.current_angle
            angle_diff = target_angle - start_angle
            
            # Автоматический расчет длительности
            if duration is None:
                # Более плавное вычисление времени
                base_time = 0.05  # уменьшили базовую задержку
                # Медленнее на малые расстояния, быстрее на большие
                proportional_time = abs(angle_diff) / 180.0 * 0.6  # увеличено до 0.6сек
                duration = base_time + proportional_time
                duration = duration / self.speed_factor
            
            # Ограничиваем длительность
            duration = max(0.03, min(duration, 1.5))  # уменьшили максимум
            
            # Больше шагов для большей плавности
            steps = max(2, int(duration * 150))  # 150 шагов в секунду!
            step_time = duration / steps
            
            # Используем smoothstep для более естественного движения
            for i in range(steps + 1):
                t = i / steps
                # Smoothstep функция - очень плавное ускорение и замедление
                t = t * t * (3 - 2 * t)
                
                # Дополнительно можно добавить небольшие easing эффекты
                # t = t < 0.5 ? 2 * t * t : -1 + (4 - 2 * t) * t  # easeInOutQuad
                
                current_angle = start_angle + angle_diff * t
                
                # Высокая точность импульса
                pulse_width = self._angle_to_pulsewidth(current_angle)
                pi.set_servo_pulsewidth(self.servo_pin, pulse_width)
                
                if i < steps:
                    # Микро-задержки для максимальной плавности
                    time.sleep(step_time)
            
            self.current_angle = target_angle
            
            print(f"Servo ultra smooth: {start_angle:.2f}° → {target_angle:.2f}° "
                  f"in {duration:.3f}s ({steps} steps)")
            return True
            
        except Exception as e:
            print(f"Error in ultra smooth move: {e}")
            return False
        finally:
            self.is_moving = False
    
    def set_angle_proportional(self, value, min_value=0.0, max_value=100.0):
        """
        Установка угла пропорционально значению (например, от слайдера)
        
        Args:
            value (float): Входное значение (например, 0-100 от слайдера)
            min_value (float): Минимальное входное значение
            max_value (float): Максимальное входное значение
        """
        # Нормализуем значение к диапазону 0-1
        normalized = (value - min_value) / (max_value - min_value)
        normalized = max(0.0, min(1.0, normalized))
        
        # Преобразуем в угол
        target_angle = self.min_angle + normalized * (self.max_angle - self.min_angle)
        
        return self.set_angle(target_angle, smooth=True)
    
    def move_by(self, delta_angle, smooth=True):
        """Относительное перемещение на дельту угла"""
        target_angle = self.current_angle + delta_angle
        return self.set_angle(target_angle, smooth)
    
    def set_speed_factor(self, factor):
        """Установка коэффициента скорости движения"""
        self.speed_factor = max(0.1, min(factor, 5.0))
        print(f"Servo speed factor set to: {self.speed_factor}")
    
    def get_angle(self):
        """Получение текущего угла с дробной частью"""
        return self.current_angle
    
    def cleanup(self):
        """Безопасная очистка ресурсов"""
        try:
            if pi:
                # Ждем завершения движения
                while self.is_moving:
                    time.sleep(0.01)
                
                # Отключаем серву
                try:
                    pi.set_servo_pulsewidth(self.servo_pin, 0)
                except:
                    pass
                
            #     # Отключаемся от демона
            #     try:
            #         pi.stop()
            #     except:
            #         pass
                
            # print("Servo (pigpio) resources cleaned up")
        except:
            pass