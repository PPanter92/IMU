# IMU

# Funktion im Robotor, warum benötigt

Roboter soll aufrecht stehen, ist jedoch in aufrechter Lage instabil, dh Roboter würde ohne Winkelmessung und Regelung umkippen und liegen. Daher Ausgleich über Motoren, die permanent Gleichgewicht widerherstellen. Roboter muss wissen, wann er aus dem Gleichgewicht gefallen ist, dies kann über Winkel gemessen werden. Ruhelage liegt bei 0° und durch Betrag und Vorzeichen des aktuell Winkels kann bestimmt werden, in welche Richtung der Roboter gekippt ist und somit entsprechend gegengesteuert werden.

Diese Lektion soll erklären, wie ein Winkel über eine IMU gemessen werden kann. IMU verfügt über Beschleunigungssensor und Gyroskop. Zu erst werden die physikalischen Fukionsweisen erklärt, dann Datenverarbeitung und am Schluss wie man damit in Arduino IDE programmieren kann.


![image](https://github.com/user-attachments/assets/c44f7af9-4e67-4d4b-81af-5215f1ecf454)


## Accelerometer /Beschleunigungssensor

### Aufgabe beim Roboter:

Der Accelerometer /Beschleunigungssensor misst die translatorische Beschleunigung - also die Beschleunigung entlang einer linearen Achse. Die IMU verfügt über 3 Accelerometer, welche jeweils senkrecht zueinander liegen, sodass Aussagen über die Beschleunigung im Raum getroffen werden können.

### aus Beschleunigung Winkel berechnen

Auch wenn der Roboter stillsteht, wirkt die Gravitationskraft auf ihn. Je nach Orientierung des Roboters im Raum, verteilt sich diese Kraft auf die 3 Achsen der IMU. Dadurch kann Orientierung bestimmt werden. Generell gilt $\sqrt{acc_x^2+acc_y^2+acc_z^2}=g$
Beim Roboter kann dies aber auf 2 Achsen reduziert werden, da die dritte immer senkrecht zum Gravitationsvektor steht und somit keinen Anteil des Gravitationsvektors aufnimmt.

![image](https://github.com/user-attachments/assets/0d127f20-096a-4870-a902-18886f3af49a)
![image](https://github.com/user-attachments/assets/6f01c241-544a-4805-9333-924c15c99b20)

**welche Achse kann reduziert werden (Beachte den Aufdruck der Achsen auf der IMU und wie sie im Roboter verbaut ist)?**

Antwort: y-Achse



Das nun reduzierte System ist $\sqrt{acc_x^2+acc_z^2}=g$




# physikalische Funktionsweise 
### Gegenstück aus der Mechanik:

![image](https://github.com/user-attachments/assets/9aa5b4c4-9f4c-4511-9e33-6236fc3883af)


Wird eine Masse, welche elastisch verbunden ist, beschleunigt, bleibt sie aufgrund ihrer Trägheit erst zurück und es entsteht eine releative Verschiebung $\Delta s$. Erst wenn die elastische Kraft $F_E$  groß genug ist um die Massenträgheit $F=m*a$  zu überwinden wird sich die Masse bewegen. Kann man die Kraft $F_E$  messen, so kann man die Verschiebung $\Delta S$ berechnen.


### Kapazität

https://moodle.ruhr-uni-bochum.de/pluginfile.php/4569060/mod_resource/content/0/Mechanische%20Sensoren.pdf

https://youtu.be/KuekQ-m9xpw



$C=\frac{\varepsilon  *A}{d}$

![image](https://github.com/user-attachments/assets/4ae052ca-bd89-4652-b14e-066fb791692f)


Die Distanz d ändert sich durch Bewegung der Seismischen Masse aufgrund ihrer Massenträgheit. 
Die Kapazitätsänderung ist nicht linear abhängig von der Änderung des
Plattenabstands. Zur Linearisierung und zur Kompensation verwendet man einen
Differentialkondensator.
Durch Kammstruktur wird d auf der einen Seite größer und auf der anderen Seite kleiner, somit ändern sich auch die Kapazitäten.


gyro


dann vorteile und nachteile der beiden, auch rauschen usw zeigen im plotter
dann kalmann filter als lösung
