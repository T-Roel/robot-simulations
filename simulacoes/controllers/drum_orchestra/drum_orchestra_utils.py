"""drum_orchestra_utils controller."""

from controller import (
    Robot, DistanceSensor, PositionSensor, TouchSensor, Motor
)
from pythonosc.udp_client import SimpleUDPClient
from time import perf_counter
from typing import List


IP = "127.0.0.1"
PORT = 57120
MAX_VELOCITY = 3.14  # Em rad/s
PI = 3.14
BPM = 115
TIME_SIGNATURE = 4, 4


def init_robot():
    """Inicializa o robô, o timestep para o mundo atual e o client de comunicação via OSC Message

    :return: objetos do robô, timestep do robô e client de comunicação via OSC Message
    """
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())
    client = SimpleUDPClient(IP, PORT)
    return robot, timestep, client


def init_motors(robot):
    """Inicializa os motores do robô recebido.

    :param robot: objeto do robô
    :return: tupla de motores da mão e das juntas do braço
    """
    hand_motors = [robot.getDevice("finger_1_joint_1"),
                   robot.getDevice("finger_2_joint_1"),
                   robot.getDevice("finger_middle_joint_1")
                   ]

    ur_motors = [robot.getDevice("shoulder_lift_joint"),
                 robot.getDevice("elbow_joint"),
                 robot.getDevice("wrist_1_joint"),
                 robot.getDevice("wrist_2_joint")
                 ]

    return hand_motors, ur_motors


def init_sensors(robot, timestep):
    """Inicializa os sensores do robô, sendo o de posição especificamente no motor do 'ombro'

    :param robot: objeto do robô
    :param timestep: timestep para o mundo atual
    :return: tupla de sensores do robô
    """
    distance_sensor = robot.getDevice("distance sensor")
    DistanceSensor.enable(distance_sensor, timestep)

    touch_sensor = robot.getDevice("force")
    TouchSensor.enable(touch_sensor, timestep)

    position_sensor = robot.getDevice("wrist_1_joint_sensor")
    PositionSensor.enable(position_sensor, timestep)

    return distance_sensor, touch_sensor, position_sensor


def switch_case(argument):
    switcher = {
        0: "WAITING",
        1: "ROTATING",
        2: "COLLISION",
        3: "ROTATING_BACK"
    }

    return switcher.get(argument, "nothing")


def set_velocity(ur_motors, speed):
    for i in range(len(ur_motors)):
        Motor.setVelocity(ur_motors[i], speed)
        i += 1


def wait_pause(time_delta):
    """Espera o tempo de pausa da nota a partir do parâmetro recebido. Tem como função parar o movimento do braço.

    :param float time_delta: tempo de pausa a ser esperado
    """
    initial_time = perf_counter()
    while True:
        if perf_counter() - initial_time >= time_delta:
            break


def thread_pause(notes, notes_counter, notes_durations):
    """Executada por uma thread, chama a função responsável pela pausa para N pausas seguidas, retornando apenas quando
    todas as pausas tiverem terminado.

    :param list notes: lista contendo todas as notas da música, para que N sucessivas pausas sejam respeitadas
    :param int notes_counter: contador indicando qual é a nota atual da lista `notes`
    :param list notes_durations: lista contendo as durações, em segundos, de todas as notas da música
    """
    try:
        while notes[notes_counter] % 10 == 0:
            notes.pop(notes_counter)
            wait_pause(notes_durations.pop(notes_counter))
    except IndexError:
        return


def get_time_signature_duration() -> float:
    """Retorna a duração em segundos de cada batida, de acordo com o BPM definido para a música

    :return: float representando a duração em segundos de cada unidade de tempo de um compasso
    """
    return 60 / BPM


def check_pause(note) -> int:
    """Checa se a nota recebida pertence ao padrão de uma pausa ou de uma nota a ser tocada. Retorna seu valor absoluto

    :param int note: nota para ser analizada
    :return: valor absoluto da nota, independente se for uma representação de pausa ou de nota a ser tocada
    """
    return int(note / 10) if note % 10 == 0 else note


def calculate_note_duration(note) -> float:
    """Calcula a duração, em segundos, da nota recebida. Utiliza o BPM e a fórmula de compasso definidos.

    :param int note: código da nota a ser calculado o tempo de duração
    :return: tempo de duração real da nota, calculado em segundos
    """
    return get_time_signature_duration() * (TIME_SIGNATURE[1] / check_pause(note))


def get_notes_duration(*notes) -> List[float]:
    """De acordo com o BPM escolhido, calcula (em segundos) quanto tempo irá durar cada nota, de acordo com seu tipo.

    :param float time_signature_duration: duração em segundos de cada unidade de tempo de um compasso
    :param int notes: lista contendo o valor representativo de cada nota a ser tocada (em última instância, da música)
    :return: lista contendo todos os tempos, em segundos, que irão durar cada a nota a ser tocada
    """
    return [calculate_note_duration(n) for n in notes]


def calculate_note_vs_distance_factor(note) -> float:
    """Calcula o fator de correção normalizado entre a nota recebida e a distância angular de movimetnação do braço.

    Para que a máxima distância angular do braço se mantenha limitada pelo valor definido em `UP_POSITION`, o máximo
    valor retornado é 1. Dessa forma, notas que duram mais que a unidade de tempo não irão considerar uma excursão
    do braço maior que o próprio `UP_POSITION`, pois irão compensar a duração da nota diminuindo a velocidade do braço.
    Para notas que possuem duração menor do que a unidade de tempo, será feita uma porcentagem para que o braço se
    movimente proporcionalmente à duração da nota. Ou seja, se a nota recebida possuir metade da duração da unidade de
    tempo, a excursão do braço será metade da total e assim por diante.

    Portanto, considerando um cenário de fórmula de compasso 4, 4, temos os seguintes exemplos:
        - Nota recebida: 4  ->  valor retornado: 1
        - Nota recebida: 2  ->  valor retornado: 1
        - Nota recebida: 8  ->  valor retornado: 0.5

    :param int note: nota a ser tocada
    :return: fator de propoção normalizado do movimento do braço
    """
    return min((TIME_SIGNATURE[1] / note), 1.0)


def calculate_angular_distance(initial_position, final_position, note_vs_distance_factor, half_note_duration) -> float:
    """Calcula a distância angular necessária para que o movimento do braço respeite o tempo da nota e a velocidade max.

    Realia dois cálculos distintos para a distância angular:
        - Excursão máxima do braço multiplicada pela fator de correção nota X distância angular;
        - Setando a velocidade do motor a 3.14 rad/s;

    Dessa forma, é utilizado o menor valor encontrado dentre as duas opções, pois caso o primeiro método resulte no
    maior valor, significa que a máxima velocidade do motor seria desrespeitada. Entretanto, se o segundo método possuir
    o maior valor, significa que não é necessário utilizar a máxima velocidade do motor, pois é possível atingir o mesmo
    tempo apenas reduzindo a velocidade e aumentando a distância.

    :param float initial_position: posição inicial do braço
    :param float final_position: posição final do braço
    :param float note_vs_distance_factor: fator de correção nota X distância angular
    :param float half_note_duration: metade do tempo total de duração da nota
    :return: distância angular a ser percorrida pelo braço, respeitando o tempo de duração da nota e a velocidade max
    """
    return min((abs(initial_position - final_position)) * note_vs_distance_factor, MAX_VELOCITY * half_note_duration)


def calculate_angular_velocity(angular_distance, half_note_duration) -> float:
    """Calcula a velocidade angular à partir do tempo de duração da nota e da distância angular informada.

    :param float angular_distance: distância angular a ser percorrida pelo braço
    :param float half_note_duration: metade do tempo total de duração da nota
    :return: velocidade angular a ser setada no motor para que a nota seja tocada corretamente
    """
    return angular_distance / half_note_duration


def get_angular_velocity(total_note_duration, note, sensor_position, target_position) -> tuple:
    """Através de funções auxiliares, calcula a velocidade e posição angulares do motor para a nota recebida

    :param float total_note_duration: tempo total de duração da nota
    :param int note: nota a ser tocada
    :param float sensor_position: valor atual do sensor de posição do motor, em rad
    :return: velocidade e posição angulares do motor para a nota recebida
    """
    note_vs_distance_factor = calculate_note_vs_distance_factor(note)
    speed = 0
    up_position = 0
    for i in range(len(target_position)):
        angular_distance = calculate_angular_distance(sensor_position, target_position[i], note_vs_distance_factor,
                                                      total_note_duration / 2)

        speed = calculate_angular_velocity(angular_distance, total_note_duration / 2)
        up_position = -1 * angular_distance

    return speed, up_position
