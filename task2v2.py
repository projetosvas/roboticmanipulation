# Escola de Engenharia de Sao Carlos - Universidade de Sao Paulo 


# Funcao de rotacao utilizando quartenions

import numpy as np # Importacao da biblioteca Numpy 
import matplotlib.pyplot as plt # Importacao da biblioteca Matplot

def quaternion(axis, angle): # Funcao que gera um quaternion  
    # A funcao recebe como entradas o eixo e o angulo de rotacao

    axis = axis/np.linalg.norm(axis) # Normaliza o vetor do eixo escolhido
    halfAngle = angle/2 # Calcula o angulo-metade
    sinHalfAngle = np.sin(halfAngle) # Calcula o seno do angulo-metade
    
    # retorna o quaternion em formato array
    return np.array([np.cos(halfAngle), # Parte real 
                    axis[0] * sinHalfAngle, # Parte complexa i
                    axis[1] * sinHalfAngle, # Parte complexa j
                    axis[2] * sinHalfAngle]) # Parte complexa k

def mulQuaternion(q1, q2): # Funcao para multiplicar quaternions
    # A funcao recebe como entrada dois quaternions

    w1, x1, y1, z1 = q1 # Decompoe o primeiro quaternion 
    w2, x2, y2, z2 = q2 # Decompoe o segundo quaternion

    # Retorna o quaternion resultante da multiplicacao em formato array
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    ])

def rotationQuaternion(v, q): # Funcao que realiza a rotacao
    # A funcao recebe o vetor que representa o corpo rigido e o
    #quaternion de rotacao

    q_conjugate = np.array([q[0], -q[1], -q[2], -q[3]]) # Calcula o conjugado
    v_quaternion = np.array([0 , v[0], v[1], v[2]]) # Transforma o vetor em um quaternion

    # Calcula o quaternion do corpo rigido rotacionado
    v_rotated = mulQuaternion(mulQuaternion(q, v_quaternion), q_conjugate)

    # Retorna apenas o vetor posicao novo ignorando a parte real
    return v_rotated[1:]

bodyPosition = np.array([2, 0, 0]) # Definicao do corpo rigido

axisRotation = np.array([0, 0, 1]) # Definicao do eixo de rotacao

theta = np.radians(90) # Definicao do angulo de rotacao

qRotation = quaternion(axisRotation, theta) # Calcula a rotacao por quaternion

newPosition = rotationQuaternion(bodyPosition, qRotation) # Nova posicao

print("Posicao Inicial: {}".format(bodyPosition)) # Retorna no cmd a posicao atual
print("Posicao apos a rotacao: {}".format(newPosition)) # Retorna no cmd a nova posicao

fig = plt.figure() # Criacao da figura
ax = fig.add_subplot(111, projection='3d') # Criacao do grafico 3D

ax.set_xlim([-3, 3]) # Definicao da escala do eixo x
ax.set_ylim([-3, 3]) # Definicao da escala do eixo y
ax.set_zlim([-3, 3]) # Definicao da escala do eixo z

ax.set_xlabel("X") # Titulo do eixo x
ax.set_ylabel("Y") # Titulo do eixo y
ax.set_zlabel("Z") # Titulo do eixo z

plt.title("3D Space") # Titulo da figura

# Plotar a posicao inicial do corpo em vermelho
ax.quiver(0,0,0,bodyPosition[0], bodyPosition[1], bodyPosition[2], color="r") 

# Plotar a posicao final do corpo em azul
ax.quiver(0,0,0,newPosition[0], newPosition[1], newPosition[2], color="b")

plt.show() # Mostrar o grafico