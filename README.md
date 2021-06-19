# **Ball and Beam (Matlab R2020a)**

**Integrantes:**  
Jaime Alejandro Condori Machaca  
Sergio Rodrigo Fernández Testa

**Docente:**  
Ing. Alexis Andrade

## Modelo 3D

![Ball and Beam](./assets/readme/ballandbeam.png)

## **Espacio de estados**

![State Space](./assets/readme/state_space.png)

## **Sistema en open-loop**

![open-loop](./assets/PLANTA.png)

## **Controladores implementados**

1. Pole placement
2. Itae

## **Parámetros**

### **Pole placement**

~~~matlab
OS = 4%
ts = 0.7[s]
~~~

### **ITAE**

~~~matlab
Wn = 7
~~~

> También se implemento el controlaor LQR, pero no se incluyó en el reporte, sus ganacias fueron:

~~~matlab
Q =  [100 0 0 0; 
        0 1 0 0;
        0 0 1 0;
        0 0 0 1];

Qext =[100 0 0 0 0; 
         0 1 0 0 0;
         0 0 1 0 0;
         0 0 0 1 0;
         0 0 0 0 2];

R = 0.0001;
~~~