# Control_robot
Se realizará el modelado y control de un brazo robot de 4 Grados de libertad (GDL). Lo que se espera obtener con este sistema es la primer etapa en un diseño basado en modelos, logrando una simulación realista del comportamiento real del mismo que pueda ser utilizada para su diseño iterativo. 

El primer paso constará en la obtención de un modelo matemático que represente la planta a controlar. En este caso, dicha planta se encontrará representada por la vinculación dinámica del brazo robótico con sus actuadores: motores de corriente continua (CC) de imanes permanentes. Para lo mismo, se dispondrá de software de diseño (Solidworks\cite{Solidworks}) y simulación (MATLAB\cite{Maltab}).  

Como paso siguiente se desarrollará un controlador robusto, mediante una combinación de controladores en espacio de estados y de controladores PID\cite{libro}. Se realizará una comparativa entre un control PID con realimentación de velocidad, y un control I-PD del mismo tipo. Debido a ello será necesaria la estimación de los valores de velocidad, para lo cual se dispondrá de observadores de estado.

Además, se simulará el modelo completo del sistema con el objetivo de analizar el correcto funcionamiento de este y su capacidad de mejora. Por último, se realizarán simulaciones con diferentes trayectorias, en donde se pondrá a prueba al control, tanto para aplicaciones comunes, como para casos extremos. 
