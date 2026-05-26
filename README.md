# Actividad 8.1 (SLAM de Lidar). Navegación y Evasión de Obstáculos

En este repositorio se presenta la implementación del control de navegación para un robot móvil de tracción diferencial. El objetivo principal de esta actividad fue lograr que el robot siguiera distintas secuencias de puntos de referencia (*waypoints*), integrando el algoritmo **Pure Pursuit** para el seguimiento de la trayectoria y el algoritmo **VFH (Vector Field Histogram)** junto con un sensor Lidar para la evasión de obstáculos.

## Parámetros Principales del Sistema
Para lograr una navegación exitosa, el comportamiento del robot y sus sensores dependen de la correcta sintonización de los siguientes parámetros y de cómo interactúan entre sí en el lazo de control:

* **`sampleTime` y `tVec`:** Definen el tiempo de muestreo y el vector de tiempo total de la simulación. Permiten discretizar el movimiento; si el tiempo total no es suficiente, el robot no terminará el recorrido.
* **`initPose`:** Establece la posición inicial (X, Y) y la orientación (theta) del robot en el mapa. Es crucial para que el algoritmo calcule correctamente la primera maniobra.
* **`lidar.scanAngles` y `lidar.maxRange`:** Configuran el campo de visión y la distancia máxima a la que el sensor detecta obstáculos. Un rango muy amplio puede asustar al robot en pasillos estrechos, mientras que un rango muy corto puede provocar choques por falta de anticipación.
* **`waypoints`:** Matriz de coordenadas cartesianas que dictan la ruta deseada.
* **`controller.LookaheadDistance`:** La distancia a la que el algoritmo Pure Pursuit "mira hacia adelante" para calcular la curva. Valores bajos hacen que siga la ruta de forma estricta, valores altos suavizan el camino pero pueden cortar curvas.
* **`controller.DesiredLinearVelocity` y `controller.MaxAngularVelocity`:** Regulan la velocidad de avance y qué tan rápido puede girar el robot sobre su propio eje. Deben estar balanceados con la visión del Lidar para dar tiempo al VFH de reaccionar antes de una colisión.

## Metodología y Escenarios de Prueba
Las pruebas se realizaron utilizando dos mapas de diferente complejidad: `exampleMap` (espacios amplios) y `complexMap` (pasillos estrechos y cajas cerradas). 

Para el **`exampleMap`**, se utilizaron valores un poco más abiertos (mayor rango de visión y mayor distancia de anticipación), ya que había mucho espacio libre para maniobrar sin riesgo de choques. 
En cambio, para el **`complexMap`**, se intentó evitar que el robot chocara con los muros. Para lograrlo en los escenarios más cerrados, fue necesario agregar **puntos intermedios (*waypoints* de apoyo)** en algunas rutas para guiar al robot por el centro de los pasillos, mientras que en otros tramos más rectos no fue necesario.

---

## Ejercicios
<img width="1328" height="357" alt="image" src="https://github.com/user-attachments/assets/1a732ac7-8d81-432c-b5f5-f38ae7533f59" />

# Ejercicio 1
<div style="display: flex; justify-content: center; gap: 10px; align-items: center;">
  <img width="400" alt="Screenshot from 2026-05-25 23-09-34" src="https://github.com/user-attachments/assets/49a569f0-2b28-47d3-ba1c-3e7cbe54dac2" />
  <img width="400" alt="Screenshot from 2026-05-25 22-31-04" src="https://github.com/user-attachments/assets/dade3837-4f5b-4c0b-9fe3-1c653f0ba1eb" />
</div>

En este primer escenario se establecieron los puntos base para formar un circuito. Se ajustaron las coordenadas dependiendo del mapa para asegurar el recorrido, para el exampleMap se movieron los puntos ya que algunos de estos chocaban independientemente del algoritmo por su posición en el mapa, por el otro lado en el complexMap los puntos estaban encima del mapa por lo cual si los sigue de manera corrceta pero es inevitable chocar con el muro.

**Para el `complexMap` (con puntos intermedios de apoyo):**
```matlab
waypoints = [
    4, 3;
    4, 6;
    9, 8;
    9, 6;
    9, 2
];
```

**Para el `exampleMap`:**
```matlab
waypoints = [
    4, 2;
    4, 6;  
    9, 6; 
    9, 2
];
```
*(Para este mapa abierto se usó una configuración con parámetros más libres que funcionaba muy bien sin necesidad de tantos ajustes finos).*

---

## Ejercicio 2
<div style="display: flex; justify-content: center; gap: 10px; align-items: center;">
  <img width="400" alt="Screenshot from 2026-05-25 23-07-34" src="https://github.com/user-attachments/assets/a9414090-23b6-4505-a242-5f6e0022ae7e" />
  <img width="400" alt="Screenshot from 2026-05-25 22-38-16" src="https://github.com/user-attachments/assets/1b61414b-efa1-4f23-8ab7-d3ebbe461bc6" />
</div>

Para el segundo recorrido, se configuró una ruta fluida de 7 puntos que ponía a prueba la capacidad del robot para realizar movimientos diagonales, al mismo que el anterior hay puntos que no lográ llegar para evitar colisiones o puntos donde roza la pared, esto debido a su posición en el mapa, aún así se demuestra buen desempeño.

**Para el `exampleMap`:**
```matlab
waypoints = [
    2, 2;
    2, 6;
    4, 8;
    8, 6;
    9, 8;
    7, 2;
    9, 3
];
```

**Para el `complexMap`:**
```matlab
waypoints = [
    2, 2;
    7, 6;  
    4, 8; 
    9, 8; 
    9, 3;
    7, 2;
    2, 6
];
```

---

## Ejercicio 3

<div style="display: flex; justify-content: center; gap: 10px; align-items: center;">
  <img width="400" alt="Screenshot from 2026-05-25 22-52-40" src="https://github.com/user-attachments/assets/26280da2-e6a5-4d59-bb8b-985923b7b398" />
  <img width="400" alt="Screenshot from 2026-05-25 22-45-41" src="https://github.com/user-attachments/assets/1d258024-871f-4bef-bde2-4d59d88f90fc" />
</div>

En el tercer ejercicio se implementó una cuadrícula secuencial de 16 *waypoints* diseñada de arriba hacia abajo para recorrer de forma continua el mapa.
**Para el `complexMap`**
```matlab
waypoints = [
    1, 2;  
    1, 1;
    2, 1;
    2, 2;
    3, 2;
    3, 1;
    4, 1;
    4, 2;
    4, 3;
    3, 3;
    4, 4;
    3, 4;
    2, 4;
    1, 4;
    1, 3;  
    2, 3];
```
**Para el `exampleMap`**
```matlab
waypoints = [
    1, 1;  
    1, 2;
    1, 3;
    1, 4;  
    2, 4;  
    2, 3;
    2, 2;
    2, 1;  
    3, 1;  
    3, 2;
    3, 3;
    3, 4;  
    4, 4;  
    4, 3;
    4, 2;
    4, 1];
```

---

## Configuración Ideal (`complexMap`)
Tras iterar con los parámetros debido a la alta densidad de obstáculos en el `complexMap`, se encontró la **configuración ideal**. Esta configuración permite que el robot pase de manera exacta por los puntos, pero logrando que los movimientos sean muy suaves y controlados gracias a la velocidad angular alta combinada con una visión enfocada.

**Parámetros clave utilizados:**
```matlab
% Initial conditions
initPose = [4; 2; 0];            

% Lidar Sensor (Visión de túnel estrecha y corta)
lidar.scanAngles = linspace(-pi/6, pi/6, 50);
lidar.maxRange = 0.5;

% Pure Pursuit Controller
controller = controllerPurePursuit;
controller.Waypoints = waypoints;
controller.LookaheadDistance = 0.4;
controller.DesiredLinearVelocity = 0.6; 
controller.MaxAngularVelocity = 90; 

% Vector Field Histogram (VFH)
vfh = controllerVFH;
vfh.DistanceLimits = [0.05 3]; 
vfh.NumAngularSectors = 36; 
vfh.HistogramThresholds = [5 10]; 
vfh.RobotRadius = 0.5;
vfh.SafetyDistance = 0.5;
vfh.MinTurningRadius = 0.25;
```

### Ciclo de Simulación
El lazo de control ejecutado en MATLAB procesa las lecturas del sensor en tiempo real para integrar el movimiento espacial del robot de la siguiente manera:

```matlab
%% Simulation loop
r = rateControl(1/sampleTime);
for idx = 2:numel(tVec) 
    
    % Obtener lecturas del sensor
    curPose = pose(:,idx-1);
    ranges = lidar(curPose);
        
    % Ejecutar algoritmos de seguimiento y evasión
    [vRef,wRef,lookAheadPt] = controller(curPose);
    targetDir = atan2(lookAheadPt(2)-curPose(2),lookAheadPt(1)-curPose(1)) - curPose(3);
    steerDir = vfh(ranges,lidar.scanAngles,targetDir);    
    
    if ~isnan(steerDir) && abs(steerDir-targetDir) > 0.1
        wRef = 0.5*steerDir;
    end
    
    % Controlar el robot y convertir a coordenadas del mundo
    velB = [vRef; 0; wRef];                   
    vel = bodyToWorld(velB, curPose);  
    
    % Integración discreta hacia adelante
    pose(:,idx) = curPose + vel*sampleTime; 
    
    % Actualizar visualización
    viz(pose(:,idx), waypoints, ranges)
    waitfor(r);
end
```
