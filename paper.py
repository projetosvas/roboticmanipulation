# Escola de Engenharia de São Carlos - Universidade de São Paulo
# Programa de Pós-Graduação em Engenharia Mecânica
# Disciplina: Manipulação Robótica
# Docente: Glauco
# Discente: Vinicius Amorim Santos

# Código para Monitoramento de Trajetória do Manipulador 3DOF

import numpy as np
import matplotlib.pyplot as plt


class Manipulador:
    def __init__(self):
        self.origem = np.array([0, 0, 0]) # Origem do manipulador
        self.j1_position = np.array([0, 0, 1]) # Posição da primeira Junta em referecia a origem
        self.j2_position = np.array([1, 0, 0]) # Posição da segunda Junta em referencia a J1
        self.j3_position = np.array([1, 0, 0]) # Posição da terceira Junta em referencia a J2

        self.j1_axis = np.array([0, 0, 1]) # Eixo de rotação da primeira Junta
        self.j2_axis = np.array([0, 1, 0]) # Eixo de rotação da segunda Junta
        self.j3_axis = np.array([0, 1, 0]) # Eixo de rotação da terceira Junta

        self.j1_oldposition = np.array([0, 0, 0]) # Posição anterior da primeira junta em referencia a origem
        self.j2_oldposition = np.array([0, 0, 0]) # Posição anterior da segunda junta em referencia a J1
        self.j3_oldposition = np.array([0, 0, 0]) # Posição anterior da terceira junta em referencia a J2


    def quaternion(self, angle, axis): # Função para gerar um quaternion
        # A função recebe o ângulo e o eixo de rotação

        axis = axis/np.linalg.norm(axis) # Normaliza o vetor eixo
        half_angle = angle/2 # Calcula o ângulo-metade
        sin_half_angle = np.sin(half_angle) # Calcula o seno do ângulo

        # retorna o quaternion em formato array
        return np.array([np.cos(half_angle), # Parte Real
                         axis[0] * sin_half_angle, # Parte Complexa i
                         axis[1] * sin_half_angle, # Parte Complexa j
                         axis[2] * sin_half_angle]) # Parte Complexa k
    

    def mul_quaternion(self, q1, q2): # Função para multiplicar quaternios
        # A função recebe como entrada dois quaternions

        w1, x1, y1, z1 = q1 # Decompoe o primeiro quaternion
        w2, x2, y2, z2 = q2 # Decompoe o segundo quaternion

        return np.array([
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
        ])
    

    def rot_quaternion(self, body, q): # Função que realiza a rotação
        # A função recebe o vetor do corpo rígido e quaternion de rotação

        q_conjugate = np.array([q[0], -q[1], -q[2], -q[3]]) # Calcula o conjugado
        body_quaternion = np.array([0,
                                    body[0],
                                    body[1],
                                    body[2]])

        body_rotated = self.mul_quaternion(self.mul_quaternion(q, body_quaternion), q_conjugate)

        return body_rotated[1:]
    

robo = Manipulador()
theta = np.array([np.radians(0),
                 np.radians(-94),
                 np.radians(-206)
                 ])

robo.j1_oldposition = robo.j1_position
robo.j1_position = robo.rot_quaternion(robo.j1_position, robo.quaternion(theta[0], robo.j1_axis))

robo.j2_oldposition = robo.j2_position
robo.j2_position = robo.rot_quaternion(robo.rot_quaternion(robo.j2_position, robo.quaternion(theta[0], robo.j1_axis)), robo.quaternion(theta[1], robo.j2_axis))
robo.j2_position = robo.j2_position + robo.j1_position

robo.j3_oldposition = robo.j3_position
robo.j3_position = robo.rot_quaternion(robo.rot_quaternion(robo.rot_quaternion(robo.j3_position, robo.quaternion(theta[0], robo.j1_axis)), robo.quaternion(theta[1], robo.j2_axis)), robo.quaternion(theta[2], robo.j3_axis))
robo.j3_position = robo.j3_position + robo.j2_position


print("Posição Inicial J1: {}".format(robo.j1_oldposition))
print("Posição Final J1: {}".format(robo.j1_position))
print("Posição Inicial J2: {}".format(robo.j2_oldposition))
print("Posição Final J2: {}".format(robo.j2_position))
print("Posição Inicial J3: {}".format(robo.j3_oldposition))
print("Posição Final J3: {}".format(robo.j3_position))

# Interface de Visualização

fig = plt.figure() # Cria uma figura
ax = fig.add_subplot(111, projection='3d')


ax.set_xlim([-2.5, 2.5]) # Definicao da escala do eixo x
ax.set_ylim([-2.5, 2.5]) # Definicao da escala do eixo y
ax.set_zlim([0, 2.5]) # Definicao da escala do eixo z

ax.set_xlabel("X") # Titulo do eixo x
ax.set_ylabel("Y") # Titulo do eixo y
ax.set_zlabel("Z") # Titulo do eixo z

plt.title("Posicionamento do Manipulador") # Titulo da figura

# Plotar a posicao inicial do corpo em vermelho
ax.quiver(0, 0, 0, robo.j1_oldposition[0], robo.j1_oldposition[1], robo.j1_oldposition[2], color="r") 

ax.quiver(robo.j1_oldposition[0],
          robo.j1_oldposition[1],
          robo.j1_oldposition[2],
          robo.j2_oldposition[0],
          robo.j2_oldposition[1],
          robo.j2_oldposition[2],
          color="r")
 
ax.quiver(robo.j1_oldposition[0] + robo.j2_oldposition[0],
          robo.j1_oldposition[1] + robo.j2_oldposition[1],
          robo.j1_oldposition[2] + robo.j2_oldposition[2],
          robo.j3_oldposition[0],
          robo.j3_oldposition[1],
          robo.j3_oldposition[2],
          color="r") 

# Plotar a posicao final do corpo em azul
ax.quiver(0, 0, 0, robo.j1_position[0], robo.j1_position[1], robo.j1_position[2], color="b")

ax.quiver(robo.j1_position[0],
          robo.j1_position[1],
          robo.j1_position[2],
          robo.j2_position[0] - robo.j1_position[0],
          robo.j2_position[1] - robo.j1_position[1],
          robo.j2_position[2] - robo.j1_position[2],
          color="b")

ax.quiver(robo.j2_position[0],
          robo.j2_position[1],
          robo.j2_position[2],
          robo.j3_position[0] - robo.j2_position[0],
          robo.j3_position[1] - robo.j2_position[1],
          robo.j3_position[2] - robo.j2_position[2],
          color="b")

plt.show() # Mostrar o grafico