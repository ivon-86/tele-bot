from flask import Flask, render_template, Response, request
import cv2
# import serial
import threading
import time
import json
import argparse

# Импортируем наш новый модуль
try:
    from control_robot import ControlServoCam
    SERVO_AVAILABLE = True
except ImportError as e:
    print(f"Warning: Could not import control_robot module: {e}")
    print("Servo control will be simulated")
    SERVO_AVAILABLE = False
except Exception as e:
    print(f"Warning: Error importing control_robot: {e}")
    SERVO_AVAILABLE = False

app = Flask(__name__)
camera = cv2.VideoCapture(0)  # веб камера

controlX, controlY = 0, 0  # глобальные переменные положения джойстика с web-страницы
servo_angle = 90  # глобальная переменная: угол сервопривода

# Инициализируем сервопривод (если доступен)
servo_cam = None
if SERVO_AVAILABLE:
    try:
        # Настройки сервопривода (можно вынести в аргументы командной строки)
        SERVO_PIN = 24  # GPIO пин для сервопривода
        servo_cam = ControlServoCam(servo_pin=SERVO_PIN)
        print("Servo camera initialized successfully")
    except Exception as e:
        print(f"Error initializing servo camera: {e}")
        servo_cam = None
else:
    print("Servo camera simulation mode")


def getFramesGenerator():
    """ Генератор фреймов для вывода в веб-страницу, тут же можно поиграть с openCV"""
    while True:
        time.sleep(0.01)    # ограничение fps (если видео тупит, можно убрать)
        success, frame = camera.read()  # Получаем фрейм с камеры
        if success:
            frame = cv2.resize(frame, (320, 240), interpolation=cv2.INTER_AREA)  # уменьшаем разрешение кадров (если видео тупит, можно уменьшить еще больше)
            # frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)   # перевод изображения в градации серого
            # _, frame = cv2.threshold(frame, 127, 255, cv2.THRESH_BINARY)  # бинаризуем изображение
            _, buffer = cv2.imencode('.jpg', frame)
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')


@app.route('/video_feed')
def video_feed():
    """ Генерируем и отправляем изображения с камеры"""
    return Response(getFramesGenerator(), mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/')
def index():
    """ Крутим html страницу """
    return render_template('index.html')


@app.route('/control')
def control():
    """ Пришел запрос на управления роботом """
    global controlX, controlY
    controlX, controlY = float(request.args.get('x')) / 100.0, float(request.args.get('y')) / 100.0
    return '', 200, {'Content-Type': 'text/plain'}


@app.route('/servo_control')
def servo_control():
    """ Пришел запрос на управление сервоприводом камеры """
    global servo_angle
    
    try:
        angle_str = request.args.get('angle')
        if angle_str:
            angle = int(angle_str)
            
            # Обновляем глобальную переменную
            servo_angle = angle
            
            # Управляем сервоприводом (если доступен)
            if servo_cam:
                success = servo_cam.set_angle(angle)
                if success:
                    print(f"Servo camera angle set to: {angle}°")
                else:
                    print(f"Failed to set servo camera angle to: {angle}°")
            else:
                # Режим симуляции
                print(f"Servo camera (simulation) angle set to: {angle}°")
            
            return '', 200, {'Content-Type': 'text/plain'}
        else:
            return 'Missing angle parameter', 400
    
    except ValueError:
        return 'Invalid angle value', 400
    except Exception as e:
        print(f"Error in servo_control: {e}")
        return 'Internal server error', 500


@app.route('/servo_status')
def servo_status():
    """ Возвращает текущий угол сервопривода """
    global servo_angle
    return json.dumps({'angle': servo_angle})


# Функция для очистки ресурсов при завершении
def cleanup_resources():
    """Очистка ресурсов при завершении работы"""
    print("Cleaning up resources...")
    
    # Освобождаем камеру
    if camera:
        camera.release()
    
    # Очищаем сервопривод
    if servo_cam:
        servo_cam.cleanup()
    
    print("Resources cleaned up")


if __name__ == '__main__':
    # пакет, посылаемый на робота
    # msg = {
    #     "speedA": 0,  # в пакете посылается скорость на левый и правый борт тележки
    #     "speedB": 0  #
    # }

    # параметры робота
    # speedScale = 0.65  # определяет скорость в процентах (0.50 = 50%) от максимальной абсолютной
    # maxAbsSpeed = 100  # максимальное абсолютное отправляемое значение скорости
    # sendFreq = 10  # слать 10 пакетов в секунду

    parser = argparse.ArgumentParser()
    parser.add_argument('-p', '--port', type=int, default=5000, help="Running port")
    parser.add_argument("-i", "--ip", type=str, default='127.0.0.1', help="Ip address")
    parser.add_argument('-s', '--serial', type=str, default='/dev/ttyUSB0', help="Serial port")
    parser.add_argument('--servo-pin', type=int, default=24, help="GPIO pin for servo camera")
    args = parser.parse_args()

    try:
        # Регистрируем обработчик завершения
        import atexit
        atexit.register(cleanup_resources)
        
        # serialPort = serial.Serial(args.serial, 9600)   # открываем uart

        # def sender():
        #     """ функция цикличной отправки пакетов по uart """
        #     global controlX, controlY
        #     while True:
        #         speedA = maxAbsSpeed * (controlY + controlX)    # преобразуем скорость робота,
        #         speedB = maxAbsSpeed * (controlY - controlX)    # в зависимости от положения джойстика

        #         speedA = max(-maxAbsSpeed, min(speedA, maxAbsSpeed))    # функция аналогичная constrain в arduino
        #         speedB = max(-maxAbsSpeed, min(speedB, maxAbsSpeed))    # функция аналогичная constrain в arduino

        #         msg["speedA"], msg["speedB"] = speedScale * speedA, speedScale * speedB     # урезаем скорость и упаковываем

        #         serialPort.write(json.dumps(msg, ensure_ascii=False).encode("utf8"))  # отправляем пакет в виде json файла
        #         time.sleep(1 / sendFreq)

        # threading.Thread(target=sender, daemon=True).start()    # запускаем тред отправки пакетов по uart с демоном

        app.run(debug=False, host=args.ip, port=args.port)   # запускаем flask приложение
        
    except KeyboardInterrupt:
        print("\nServer stopped by user")
    finally:
        cleanup_resources()