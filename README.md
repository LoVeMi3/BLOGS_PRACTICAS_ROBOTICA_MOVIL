# BLOGS_PRACTICAS_ROBOTICA_MOVIL


## BASIC VACUUM CLEANER
	
Esta primera práctica se trataba de programar una aspiradora pseudoautónoma, que
recorrería una habitación o casa hasta llegar al 100%. Se trataría de seguir
un movimiento en espiral para cubrir la mayor superficie posible, teniendo en 
cuenta los obstáculos que la aspiradora pudiese encontrar, como paredes o mesas.
La dirección de los giros que haría la aspiradora se decidiría de forma aleatoria, 
estos causados por un obstáculo detectado.
El objetivo es recorrer, "limpiar", el 100% de la habitación o casa, sin depender 
de qué mapa se utilice.
	
El primer paso que realicé fue buscar código de mi propiedad, de otras prácticas 
en las que ya había utilizado la misma lógica de limpiar una habitación, sobre todo
porque hicimos una práctica en grupo de un robot de servicios de un hotel. Ya 
con la lógica del código en mente, lo que más sencillo que se me planteaba era un 
switch (if/elif) de estados dentro de un while. Todo esto dentro de la función 
exercise() dada por la plantilla.
	
El while en exercise es necesario para la recogida automática y continua de los 
180 valores del laser, comprobar que están todos y que no son nulos. Después de 
eso, hay que dividir el cono de visión del laser en tres grupos: el centro, la 
derecha y la izquierda, siendo estos los que señalan si se detecta un obstáculo 
en cualquiera de las tres direcciones. Este cono de visión lo podemos definir en 
aproximación sabiendo que el valor del centro es 90, pero a la hora de definir 
los valores "límites" que definirían los tres grupos, tuve que aumentar de 75 a 
60 y de 105 a 120 (los originales) porque se me quedaba en un campo de visión 
corto que hacía que el robot se quedara enganchado en paredes que tenía detrás. 
Después, si cualquiera de los valores de estas direcciones está por debajo del 
mínimo de detección de un obstáculo, la variable es true y empezamos la lógica 
del switch de estados.

En el switch (condicional) tenemos cuatro estados: SPIRAL, FORWARD, BACKUP, TURN. 
En el estado SPIRAL nos centramos en ir calculando, dependiendo del número de 
ticks que han pasado, el valor del radio que tiene que ir tomando la espiral a 
medida que aumentan los ticks. El estado FORWARD trata simplemente de que los 
motores vayan en línea recta hacia delante, y se cambia a él cuando ya han pasado 
suficientes ticks. Estos dos estados tienen en cuenta si se detecta un obstáculo 
y cambian al estado de BACKUP, dónde se elige de forma random en qué dirección 
gira luego el robot tras ir hacia atrás. Tras eso, se cambia al estado TURN, 
donde se gira y se vuelve a FORWARD.

Dónde más dificulted encontré fue en la división de los grupos de visión del cono 
de los valores del láser, no sabía qué pasaba exactamente, hasta que me recomendaron 
aumentar el rango de los valores. Lo que también me costó fueron los ticks, no 
sabía que los necesitaba, y no caía que me hacían falta para resolver el hecho 
de que el robot en un punto se quedaba haciendo el mismo recorrido de la habitación 
y no recorría el resto de la habitación, se quedaba siguiendo la pared. Llegaba 
su límite de aumento del radio de la espiral, encontraba todo el rato el mismo 
obstáculo que era la pared, y se quedaba ahí. Los ticks completaban la lógica 
de que girara de forma random al detectar un obstáculo.

Para finalizar, el vídeo original me ha dorado 100 minutos, cuando llegué a esa 
marca lo paré aun cuando sólo estaba al 84% porque me dí cuenta que el mapa que 
estaba percibiendo el robot era el de la pantalla de en medio, dónde las habitaciones 
son más estrechas, entonces en la pantalla de arriba por eso se ve como que detecta 
una pared antes de acercarse a ella. Y pues, sinceramente, paré el video.

El vídeo

(https://youtu.be/el7jMzhjojQ)


## FOLLOW LINE

En esta segunda práctica, de lo que se trataba era de hacer un siguelíneas pero 
en formato de un coche de Fórmula 1, probándolo en distintos circuitos de Fórmula 
1.

Mi primer paso fue declarar las variables de las distintas velocidades para los 
diferentes movimientos que podría hacer el coche: si busca la línea, si la sigue 
de forma recta o si la sigue en una curva. También declaré los dos estados entre 
los que se mueve el robot, las varibles de tiempo, la de error y la de la última 
dirección; siendo estás globales y claves para la lógica del siguelíneas.

La lógica que tenía en mente que siguiera el siguelíneas era básicamente dependiente 
de si se veía la línea o no. Cuando la línea fuera visible, el coche determinaría 
a partir del error en la imagen si nos encontrábamos en una curva o no. Esto también 
dependía del threshold que declararíamos como mínimo para si lo que era detectado 
era una curva o una recta. Por otro lado, cuando la línea no fuera visible, cuando 
el error fuera None, empezaríamos un "timer" antes de ejecutar la función de búsqueda 
de la línea, que iría contando los segundos desde que se ha perdido la línea y, 
dependiendo de ese tiempo, buscaría la línea de una forma u otra.

Sinceramente, lo que más se me hizo complicado al principio fue el paso de RGB a 
HSV porque, genuinamente, no entendía cómo funcionaba la sintaxis del cv2. Con 
ayuda de la página web que ustedes enseñaron y el comentario de los valores de RGB 
entre los que se movía el rojo, entendí cómo funcionaba. Modifiqué la imagen que 
captaría y utilizaría el coche; recorté la imagen al 40% de la parte inferior 
(dónde se encontraría la línea), y como sólo tenía en cuenta el color rojo, en la 
pantalla se ve la línea en blanco y todo lo demás (que no es rojo) se ve en negro. 
El contraste es clave para encontrar la línea siempre.

Para finalizar, estuve jugando con los valores de las velocidades de giro y de 
línea recta. Me fijé que si había mucha distancia entre ambos valores, después de 
hacer un giro (cerrado o no), al volver a la línea recta, al coche le costaba mucho 
estabilizarse otra vez, sin oscilaciones. Si aumento la velocidad en línea recta, 
tengo que tener cuidado aumentando también la velocidad en curvas.
Bueno, esta parte es realmente la más complicada de toda la práctica. Todas las 
variables que hay que tener en cuenta, los cálculos, el manejo de las oscilaciones 
dependiendo de cuánto aumentes la velocidad en línea recta, el tener cuidado con 
aumentar la velocidad de curva porque de repente te das con la pared de la simulación. 
Ha sido un poco pesadilla, sobre todo cuando tenemos en cuenta que tienes dos 
velocidades distintas pero un sólo error, y tienes que tener en cuenta las K's 
distintas, la última dirección y la última curva en la que estabas. Y cuidado 
con el área de la imagen que estás teniendo en cuenta, porque a veces ve la siguiente 
curva que hay delante y se adelanta a ésta.

En conclusión, los siguelíneas no son lo mío. No hay año en esta carrera que no 
lo pase mal con un siguelíneas, y mira que es de las prácticas más sencilla. Pero 
es que son los PID's. Y el error. Sumándole las distintas velocidades, mucho dolor 
de cabeza. Y esta vez lo he hecho sólo en python, porque si no, no entendía el paso 
de RGB a HSV. Siento que la conclusión sea yo quejándome, pero de verdad que lo 
he intentado con las K's.

El vídeo del circuito simple
(https://youtu.be/7Dr84OfwMbI)

El video del circuito de Montmelo
(https://youtu.be/bnBYwmm2mJw)

El video del circuito de Montreal
(https://youtu.be/27uhqMzVJXM)

El video del circuito de Nurtburing
(https://youtu.be/JlvWDe4Aaug)


## OBSTACLE AVOIDANCE

Esta práctica trataba del mismo coche de la anterior, que simplemente va haciendo el 
circuito pero guiándose siguiendo una lista de coordenadas objetivo, pasando el coche 
por cada uno. Estos "target", sus coordenadas funcionan como 
vectores atractivos para el robot, mientras que los obstáculos que se van encontrando, y 
las paredes del circuito, funcionan commo vectores repulsivos. La suma de los vectores 
atractivos y repulsivos crean el vector resultante por el que se guía el robot para 
continuar el circuito sin chocarse con los obstáculos.

En esta práctica, lo que más me ha costado al principio, ha sido entender el orden en el 
que tenía que coger los datos y las coordenadas. Al principio tenía el bucle principal 
para que lo primero que detectara fuera si había obstáculos, entonces tenía una función que 
detectara los obstáculos, los esquivara y me devolviera las coordenadas del obstáculo. Pero 
necesitaba el vector de repulsión para crear el vector resultante, e hice una función que me 
lo sacara. Y de repente tenía dos funciones que hacían prácticamente lo mismo. Esto me pasa 
por coger mi código de Arquitectura Software. Bueno. Mezcle las dos funciones en una, que a 
partir de los datos del laser detectara si había un obstáculo, y sacar de ahí solamente el 
vector repulsivo. Me líe yo sola.

Sorprendentement, ajustar la velocidad y la k del vector resultante fue más sencilla esta vez 
porque no buscaba mucha velocidad en los motores. Con lo que estuve jugando es con la distancia 
mínima a la que se detectaba un obstáculo. Si era demasiada (2.5) y el robot se quedaba entre 
un coche y una pared, e intenta salir de ahí yendo hacia delante, se quedaba atascado y zigzagueando 
por los dos obstáculos a su alrededor. Entonces la reduje a 2.3. Sólo es un cambio significativo para 
mí, pero intenté de alguna forma que cuando detectara un obstáculo en vez de girarse siempre al 
mismo sitio, que girara a donde habría más hueco, por ejemplo con lo de multiplicar por -1 ó 1 
a la velocidad angular. Pero no me salía, y simplemente reduje la distancia mínima, y funcionaba, así que \^·_·^/.

En conclusión, esta práctica fue más fácil de lo que pensaba al principio, cuando me hice un lío 
con lo de mezclar el siguelíneas y el detectar obstáculos al mismo tiempo, pero al final era más 
sencillo de lo que pensaba. Y menos mal, porque no me salía.

Vídeo:
(https://youtu.be/SQmn1ghYVmU)



## GLOBAL NAVIGATION

Definitivamente, confirmamos que esta práctica ha sido la que más difícil se me 
ha hecho. Se trataba de trabajar con la navegación global, en la que tenemos un 
mapa aprendido, con sus waypoints, dividido en celdas y/o coordenadas métricas. 
Por él se va moviendo el robot en dirección al objetivo (target) que le señalamos 
al clickar en él en el mapa de la simulación. 

El primer paso que dí fue entender los lugares que debían ocupar el control y la 
planificación de la navegación. El control del robot, el que se mueva siguiendo 
la ruta planificada, que gire o que vaya recto, va dentro del bucle while, según 
la frecuencia ideal, lo que ya hemos hecho antes. Pero el dilema ahora es la 
planificación, que sólo se hace una vez, fuera del bucle. No se puede hacer la 
planificación dentro del bucle porque entonces ya no es global y no tendría sentido 
trabajar con un mapa que nos ayuda a calcular la mejor ruta.

A continuuación, cree una función de planificación a partir de las variables de 
la matriz del mapa, y las celdas de origen y del objetivo. La función de planificación 
trata básicamente de asignarles valores numéricos a las celdas de toda la matriz 
del mapa según lo cerca que estén del objetivo desde el origen. Primero, todas las 
celdas valen -1. Segundo, según la dirección que puede tomar el coche y la distancia 
en celdas a la pared más cercana, creamos el mapa de penalizaciones (Djikstra), 
calculando dist teniendo en cuenta el coste de cada paso en el mapa. Estos costes 
de cada celda van del objetivo al origen del coche, cuanto menos coste más cerca 
del objetivo nos encontramos. El coste de la celda depende también de si se encuentra 
más cerca o no de la pared (me atravesaba algunas si no). Y lo tercero y último 
de la función de planficación es el cálculo de la mejor ruta a seguir, dependiendo 
de los valores que les hemos asignado antes a las celdas del mapa. Y devuelve 
ambas.
En la función de planificación me ayudo de funciones como step_cost, para calcular 
el coste nuevo de la celda que estamos valorando. Y también utilizo la función 
obstacle_distance, que nos devuelve las celdas que están pegadas a las paredes.

Siendo realistas, la parte de planificación no funcionaría sin comprobar que se 
selecciona una X en el mapa, una celda objetivo a la que mandar el coche; y tampoco 
sin sacar la celda origen de la que parte el coche. Hay que hacer también las debidas 
comprobaciones, como que tenemos una ruta que seguir. Eso sí, la parte de numpy 
sí que no sabía cómo se hacía, no sabía cómo hacer para mostrar el mapa con el 
gradiente. Y para la prósima vez, tengo que acordarme que el método worldToGrid 
devuelve (columna, fila), no (fila, columna).

Apunte final, la función de make_cell_wp me ayudaron unos compaleros a hacerlo 
porque no entendía por qué me atravesaba las paredes al girar justo antes de una 
esquina, en la que tenía que girar realmente. Lo que hace es ir marcándolos uno a 
uno, dependiendo de la ruta que se le pase y la cantidad de pasos (10) que va a 
ir haciendo.

Ha sido difícil, mucho. Menos mal que nos dieron una tarde más. Eso sí, no renuevo 
el valor de la celda de target una vez llego a el objetivo, lo guarda. Si quieren 
probar un nuevo punto, tendrán que cerrar el docker y volver a abrirlo para seleccionar 
el nuevo objetivo del robot.



Vídeos de la práctica (con V_STRAIGHT = 5 y V_TURN = 3):

(https://youtu.be/JKGk0_zgj8Q)

(https://youtu.be/HO76WYj6n6k)


Vídeos de la práctica (con V_STRAIGHT = 10 y V_TURN = 6):

(https://youtu.be/pH0hZh26peU)

(https://youtu.be/x9GNVnJFS3U)
