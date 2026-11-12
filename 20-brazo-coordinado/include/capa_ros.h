#pragma once
// Capa micro-ROS de 20 (solo con -DUSAR_MICROROS, entorno `ros` de platformio.ini). Mismos comandos que por Serial.
//
// Suscribe  /brazo/comando    std_msgs/String  una línea como en el monitor: jv a1 a2 a3, h, s, x, mv q1 q2 q3,
//                                              pz a|c|0|<us>, mn a|b|0|<us>, vel ..., c ...
// Publica   /brazo/estado     std_msgs/String  q=q1,q2,q3;e=e1,e2,e3;c=111;fc=000;pz=<us>;mn=<us>     10 Hz
//                                              (e: 0 suelta, 1 yendo, 2 llegó, 3 firme, 4 frenada, 5 mov, 6 homing;
//                                              pz/mn -1 = suelto)
//           /brazo/respuesta  std_msgs/String  lo que el monitor serial mostraría
// Sin agente: "s" (las tres se sostienen); el joystick (jv) ya frena solo a los 300 ms sin comandos.
void rosBegin();
void rosLoop();
