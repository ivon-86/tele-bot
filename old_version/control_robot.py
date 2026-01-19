# control_robot.py
"""
Модуль для управления аппаратными компонентами робота:
- Сервопривод камеры (управление через GPIO с soft PWM)
- Моторы гусеничной платформы (будет добавлено позже)
- Энкодеры (будет добавлено позже)
"""

import RPi.GPIO as GPIO
import time

class ControlServoCam:
    """
    Класс для управления сервоприводом камеры через GPIO
    Использует software PWM, так как аппаратный ШИМ занят под двигатели
    """
    
    def __init__(self, servo_pin=24, min_angle=0, max_angle=180, default_angle=90):
        """
        Инициализация сервопривода камеры
        
        Args:
            servo_pin (int): GPIO пин для управления сервой
            min_angle (int): Минимальный угол поворота (градусы)
            max_angle (int): Максимальный угол поворота (градусы)
            default_angle (int): Угол по умолчанию (градусы)
        """
        self.servo_pin = servo_pin
        self.min_angle = min_angle
        self.max_angle = max_angle
        self.current_angle = default_angle
        
        # Настройка GPIO
        GPIO.setmode(GPIO.BCM)  # Используем нумерацию BCM
        GPIO.setup(self.servo_pin, GPIO.OUT)
        
        # Создаем software PWM с частотой 50Hz (стандартно для сервоприводов)
        self.pwm = GPIO.PWM(self.servo_pin, 50)
        self.pwm.start(self._angle_to_duty_cycle(default_angle))
        
        # Устанавливаем начальный угол
        self.set_angle(default_angle)
        
        print(f"Servo camera initialized on GPIO {servo_pin}")
        print(f"Angle range: {min_angle}° - {max_angle}°")
    
    def _angle_to_duty_cycle(self, angle):
        """
        Преобразование угла в коэффициент заполнения ШИМ
        
        Для сервоприводов обычно:
        - 0° = 2.5% duty cycle (1ms импульс при 20ms периоде)
        - 90° = 7.5% duty cycle (1.5ms импульс)
        - 180° = 12.5% duty cycle (2ms импульс)
        
        Args:
            angle (int): Угол в градусах (0-180)
            
        Returns:
            float: Коэффициент заполнения ШИМ (2.5-12.5%)
        """
        # Ограничиваем угол в допустимых пределах
        angle = max(self.min_angle, min(angle, self.max_angle))
        
        # Преобразуем угол в коэффициент заполнения
        # Линейное преобразование: угол 0-180 → 2.5-12.5%
        duty_cycle = 2.5 + (angle / 180.0) * 10.0
        return duty_cycle
    
    def set_angle(self, angle):
        """
        Установка угла сервопривода
        
        Args:
            angle (int): Угол в градусах (0-180)
            
        Returns:
            bool: True если угол установлен успешно
        """
        try:
            # Проверяем границы угла
            if angle < self.min_angle or angle > self.max_angle:
                print(f"Warning: Angle {angle}° is out of range ({self.min_angle}°-{self.max_angle}°)")
                angle = max(self.min_angle, min(angle, self.max_angle))
            
            # Преобразуем угол в коэффициент заполнения
            duty_cycle = self._angle_to_duty_cycle(angle)
            
            # Устанавливаем новый коэффициент заполнения
            self.pwm.ChangeDutyCycle(duty_cycle)
            
            # Сохраняем текущий угол
            self.current_angle = angle

            # Даем серве время на перемещение (50ms достаточно для большинства сервоприводов)
            time.sleep(0.05)

            # ОТКЛЮЧАЕМ PWM для фиксации!
            self.pwm.ChangeDutyCycle(0)
            
            
            print(f"Servo camera angle set to: {angle}° (duty cycle: {duty_cycle:.1f}%)")
            return True
            
        except Exception as e:
            print(f"Error setting servo angle: {e}")
            return False
    
    def get_angle(self):
        """
        Получение текущего угла сервопривода
        
        Returns:
            int: Текущий угол в градусах
        """
        return self.current_angle
    
    def center(self):
        """
        Установка сервопривода в центральное положение
        
        Returns:
            bool: True если операция выполнена успешно
        """
        center_angle = (self.min_angle + self.max_angle) // 2
        return self.set_angle(center_angle)
    
    def sweep_test(self, delay=0.1):
        """
        Тестовый прогон сервопривода по всему диапазону
        
        Args:
            delay (float): Задержка между шагами в секундах
        """
        print("Starting servo sweep test...")
        
        # От минимума до максимума
        for angle in range(self.min_angle, self.max_angle + 1, 5):
            self.set_angle(angle)
            time.sleep(delay)
        
        # От максимума до минимума
        for angle in range(self.max_angle, self.min_angle - 1, -5):
            self.set_angle(angle)
            time.sleep(delay)
        
        # Возврат в центр
        self.center()
        print("Servo sweep test completed.")
    
    def cleanup(self):
        """
        Очистка ресурсов GPIO
        """
        print("Cleaning up servo camera resources...")
        self.pwm.stop()
        GPIO.cleanup(self.servo_pin)
        print("Servo camera resources cleaned up.")


# Дополнительные классы (заглушки, будут реализованы позже)
class ControlMotors:
    """Класс для управления моторами гусеничной платформы"""
    def __init__(self):
        print("Motors controller initialized (placeholder)")
    
    def cleanup(self):
        print("Motors controller cleaned up")


class ControlEncoders:
    """Класс для работы с энкодерами"""
    def __init__(self):
        print("Encoders controller initialized (placeholder)")
    
    def cleanup(self):
        print("Encoders controller cleaned up")

#fjghdfghfjdgdf
# Пример использования (для тестирования)
if __name__ == "__main__":
    try:
        # Создаем экземпляр сервопривода
        # Примечание: на Raspberry Pi нужно запускать с правами sudo
        servo_cam = ControlServoCam(servo_pin=24)
        
        # Тестируем основные функции
        servo_cam.set_angle(0)
        time.sleep(4)
        
        servo_cam.set_angle(90)
        time.sleep(4)
        
        servo_cam.set_angle(180)
        time.sleep(4)
        
        # servo_cam.set_angle(70)
        # time.sleep(4)

        # servo_cam.set_angle(33)
        # time.sleep(4)

        servo_cam.center()
        time.sleep(4)
        
        # Можно протестировать прогон по диапазону (раскомментировать при необходимости)
        servo_cam.sweep_test(delay=0.3)
        
    except KeyboardInterrupt:
        print("\nПрограмма прервана пользователем")
    except Exception as e:
        print(f"Ошибка: {e}")
    finally:
        # Всегда очищаем ресурсы
        servo_cam.cleanup()