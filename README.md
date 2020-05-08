Material necesario para dispositivo GPS open source

Arduino pro micro(por su velocidad de procesamiento a 16 Mhz y su versatil tamaño).

Modulo GPS neo6m v2(permite obtener: longitud, latitud, altitud, hora y fecha).

Modulo SIM800L(establece conexión a una red especificada por el usuario para la comunicación).

Tarjeta SIM sin PIN lock(permite el uso de datos para entablar comunicaciones mediante operadores distintos).

Antena GPS y GSMR(permiten a los modulos sim800l y neo6m v2 conectarse a sus respectivos dispositivos).

Arduino pro micro es una tarjeta de desarrollo basada en un sistema de bootloader que permite
simplificar la programación, a su vez procesa la información obtenida del modulo neo6m, configura
el modulo sim800l para ser enviada a un servidor remoto donde se decodificará obtiendo la posicion exacta.

Un GPS por sus siglas "Global Positioning System” permite triangular una posicion en cualquier
parte del globo terraaqueo, es por esto que se vuelve indisénsable para el prpyecto.

El modulo sim800l permite el uso de datos mediante la conexion privada a un operador usando un APN,
nombre de usuario y contraseña, este permite el envío de las ubicaciones procesadas por el modulo neo6m v2.

