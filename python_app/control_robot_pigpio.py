# control_robot_pigpio.py - улучшенная версия
import pigpio
import time
import math

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
        
        # Подключаемся к pigpio демону
        self.pi = pigpio.pi()
        
        if not self.pi.connected:
            raise RuntimeError("Cannot connect to pigpio daemon. Run: sudo pigpiod")
        
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
            self.pi.set_servo_pulsewidth(self.servo_pin, pulse_width)
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
        """Плавное движение к целевому углу"""
        if self.is_moving:
            print("Servo is already moving")
            return False
        
        self.is_moving = True
        
        try:
            start_angle = self.current_angle
            angle_diff = target_angle - start_angle
            
            # Автоматический расчет длительности
            if duration is None:
                # Базовое время + пропорциональное разнице углов
                base_time = 0.1  # базовая задержка
                proportional_time = abs(angle_diff) / 180.0 * 0.4  # до 0.4сек на полный оборот
                duration = base_time + proportional_time
                duration = duration / self.speed_factor  # учитываем коэффициент скорости
            
            # Ограничиваем минимальную и максимальную длительность
            duration = max(0.05, min(duration, 2.0))
            
            # Количество шагов для плавности
            steps = max(2, int(duration * 100))  # 100 шагов в секунду
            step_time = duration / steps
            
            # Плавная интерполяция (можно менять функцию интерполяции)
            for i in range(steps + 1):
                # Линейная интерполяция
                t = i / steps
                current_angle = start_angle + angle_diff * t
                
                # Можно использовать ease-in-out для более естественного движения:
                t = t * t * (3 - 2 * t)  # smoothstep
                current_angle = start_angle + angle_diff * t
                
                pulse_width = self._angle_to_pulsewidth(current_angle)
                self.pi.set_servo_pulsewidth(self.servo_pin, pulse_width)
                
                if i < steps:  # Не спим на последнем шаге
                    time.sleep(step_time)
            
            # Фиксируем конечную позицию
            self.current_angle = target_angle
            
            print(f"Servo smoothly moved: {start_angle:.1f}° → {target_angle:.1f}° "
                  f"in {duration:.2f}s ({steps} steps)")
            return True
            
        except Exception as e:
            print(f"Error in smooth move: {e}")
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
            if hasattr(self, 'pi') and self.pi:
                # Ждем завершения движения
                while self.is_moving:
                    time.sleep(0.01)
                
                # Отключаем серву
                try:
                    self.pi.set_servo_pulsewidth(self.servo_pin, 0)
                except:
                    pass
                
                # Отключаемся от демона
                try:
                    self.pi.stop()
                except:
                    pass
                
            print("Servo (pigpio) resources cleaned up")
        except:
            pass