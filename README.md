# IMU

# Funktion im Robotor, warum benötigt

Roboter soll aufrecht stehen, ist jedoch in aufrechter Lage instabil, dh Roboter würde ohne Winkelmessung und Regelung umkippen und liegen. Daher Ausgleich über Motoren, die permanent Gleichgewicht widerherstellen. Roboter muss wissen, wann er aus dem Gleichgewicht gefallen ist, dies kann über Winkel gemessen werden. Ruhelage liegt bei 0° und durch Betrag und Vorzeichen des aktuell Winkels kann bestimmt werden, in welche Richtung der Roboter gekippt ist und somit entsprechend gegengesteuert werden.

**video / gif wie roboter einfach umfällt vs wie er sich stabilisiert**

Diese Lektion soll erklären, wie ein Winkel über eine IMU gemessen werden kann. IMU verfügt über Beschleunigungssensor und Gyroskop. Zu erst werden die physikalischen Fukionsweisen erklärt, dann Datenverarbeitung und am Schluss wie man damit in Arduino IDE programmieren kann.


![image](https://github.com/user-attachments/assets/c44f7af9-4e67-4d4b-81af-5215f1ecf454)


## Accelerometer /Beschleunigungssensor

### Aufgabe beim Roboter:

Der Accelerometer /Beschleunigungssensor misst die translatorische Beschleunigung - also die Beschleunigung entlang einer linearen Achse. Die IMU verfügt über 3 Accelerometer, welche jeweils senkrecht zueinander liegen, sodass Aussagen über die Beschleunigung im Raum getroffen werden können.

### aus Beschleunigung Winkel berechnen

Selbst wenn der Roboter still steht, wirken Kräfte auf ihn, vor allem die Gravitationskraft, welche wir uns zur Nutze machen können, um seinen Neigungswinkel zu besitmmen. Je nach Orientierung des Roboters im Raum, verteilt sich diese Kraft auf die 3 Achsen der IMU. Dadurch kann Orientierung bestimmt werden. Generell gilt $\sqrt{acc_x^2+acc_y^2+acc_z^2}=g$

![image](https://github.com/user-attachments/assets/c2be409f-ff3c-4928-a4fe-9e8a07fe7704)
![image](https://github.com/user-attachments/assets/37c935f7-4180-4ed1-b548-0607257865b6)


Beim Roboter kann dies aber auf 2 Achsen reduziert werden, da die dritte im normalen Betrieb immer senkrecht zum Gravitationsvektor steht und somit keinen Anteil des Gravitationsvektors aufnimmt.

![image](https://github.com/user-attachments/assets/7b6ebb64-442b-40c2-8db8-4366414f648b)



**welche Achse kann reduziert werden (Beachte den Aufdruck der Achsen auf der IMU und wie sie im Roboter verbaut ist)?**

Antwort: y-Achse

![image](https://github.com/user-attachments/assets/1a6d81c9-1d3b-47fe-bd09-4da7b1181330)


Das nun reduzierte System ist $\sqrt{acc_x^2+acc_z^2}=g$
Wenn  $acc_x$ und $acc_z$ gemessen werden, kann Mithilfe von simplen trigonomerischen Zusammenhängen der Winkel $\theta$  berechnet werden.

![image](https://github.com/user-attachments/assets/ace00732-153f-469b-aea0-73f26330c709)
![image](https://github.com/user-attachments/assets/ce4f99a1-2da3-4e9b-abde-91d473c19b7a)

Überlege welche trig Funktion grundsätzlich verwendet Werden kann, um aus den bekannten Werten  $acc_x$ und $acc_z$ der Winkel $\theta$  zu bestimmen


Es ist von Vorteil, wenn die Nulllage bei $\theta=0$ liegt, da Auslenkungen um diese Nulllage bedeuten, dass $\theta$  von 0 bis +90 bzw 0 bis -90° geht
wie lautet dann die Berechnung für $\theta$?

Anwort: atan2(z,-x)



# physikalische Funktionsweise 
## Accelerometer
### Gegenstück aus der Mechanik:

![image](https://github.com/user-attachments/assets/9aa5b4c4-9f4c-4511-9e33-6236fc3883af)

Eine Inertial Measurement Unit (IMU) enthält typischerweise einen Beschleunigungssensor (Accelerometer) und ein Gyroskop. Der grundlegende physikalische Hintergrund eines Accelerometers basiert auf dem Konzept der seismischen Masse.

Eine seismische Masse ist eine Masse, die nicht starr, sondern elastisch mit einem System  verbunden ist. Wenn diese seismische Masse beschleunigt wird, bleibt sie aufgrund ihrer Trägheit zunächst zurück, was zu einer relativen Verschiebung Δs zwischen der Masse und dem System führt. Diese Verschiebung erzeugt eine elastische Kraft $F_E$​, die proportional zur Verschiebung ist.

Die Trägheitskraft $F=m*a$ (wobei m die Masse und a die Beschleunigung ist) wirkt der elastischen Kraft entgegen. Die seismische Masse bewegt sich erst dann, wenn die elastische Kraft $F_E$ groß genug ist, um die Trägheitskraft zu überwinden.

Durch die Messung der elastischen Kraft $F_E$ kann die Beschleunigung a berechnet werden. Die elastische Kraft ist proportional zur Verschiebung $\Delta s$, und durch die Kenntnis der Federkonstanten des Systems kann die Beschleunigung bestimmt werden. Dies ist das grundlegende Prinzip, auf dem ein Accelerometer basiert.


### Kapazität
![image](https://github.com/user-attachments/assets/c25d00f4-b8bc-434b-8ec1-bac817eb3969)


Der tatsächliche, elektro-mechanische Sensor Berechnet die besteht aus zwei Elektroden. Eine davon ist fest verbaut und die andere kann frei an einem Anker als seismische Masse schwingen. 
Die Kapazität $C$ eines Kondensators folgt der Formel $C=\frac{\varepsilon  *A}{d}$
Die Distanz d zwischen den Kondensatorplatten ändert sich durch Bewegung der Seismischen Masse (aufgrund ihrer Massenträgheit). 

Für eine genauere Messung wird nicht nur ein Kondensator verwendet, sondern mehrere, welche über den kammförmigen Aufbau der Elektroden entstehen. Einen solchen Aufbau nennt man Differentialkondensator. Dieser vergleicht die Kapazitäten von mehreren Kondensatoren. Wird der Abstand $D_1$  auf der einen Seite größer wird der Abstand $D_2$ auf der anderen Seite kleiner. Somit ändern sich auch die beiden Kapazitäten und ihre Verhältnisse zueinander. Dies ermöglicht eine genauere Messung der Bewegung und damit der Beschleunigung.

![image](https://github.com/user-attachments/assets/d1018953-887a-4568-b061-16cb35681aed)



## Gyroskop 

Ein Gyroskop ist ein Sensor, der die Winkelgeschwindigkeit misst, also wie schnell sich ein Objekt um eine Achse dreht. Der grundlegende physikalische Hintergrund eines Gyroskops basiert auf dem Prinzip der Kreiselstabilität und der Corioliskraft.

Ein typisches Gyroskop enthält eine rotierende Masse, die als Kreisel bezeichnet wird. Wenn der Kreisel rotiert, erzeugt er aufgrund seiner Trägheit einen Drehimpuls. Dieser Drehimpuls bleibt konstant, solange keine äußeren Drehmomente auf den Kreisel wirken. Wenn das Gyroskop gedreht wird, wirkt eine Corioliskraft auf die rotierende Masse.

Die Corioliskraft FC​ ist proportional zur Winkelgeschwindigkeit ω und zur Geschwindigkeit der rotierenden Masse v. Sie kann durch die folgende Formel beschrieben werden:

FC​=2⋅m⋅v⋅ω

wobei m die Masse der rotierenden Scheibe ist.

Diese Corioliskraft führt zu einer Verschiebung der rotierenden Masse, die von Sensoren im Gyroskop gemessen wird. Die gemessene Verschiebung ist proportional zur Winkelgeschwindigkeit ω. Durch die Kenntnis der Masse und der Geschwindigkeit der rotierenden Scheibe kann die Winkelgeschwindigkeit berechnet werden.

Ein Gyroskop nutzt also die Corioliskraft, die auf eine rotierende Masse wirkt, um die Winkelgeschwindigkeit zu messen. Diese Messung ermöglicht es, die Drehbewegung eines Objekts präzise zu erfassen und ist ein wesentlicher Bestandteil der Inertial Measurement Unit (IMU).



# Ausgangs-Signal des Sensors

***informationen und erklärung***

Die von uns eingesetzte IMU, die MPU-6050, gibt ihre Messwerte als digitale Signale über die I2C-Schnittstelle aus. Die I2C-Schnittstelle ermöglicht eine einfache Kommunikation zwischen der MPU-6050 und dem Microcontroller. Die MPU-6050 hat eine feste I2C-Adresse, über die sie angesprochen wird. Die Messwerte der Sensoren werden in speziellen Registern gespeichert, die über I2C ausgelesen werden können.

Die Rohdaten, die von der MPU-6050 ausgegeben werden, sind 16-Bit-Werte, die die Beschleunigung in den jeweiligen Achsen repräsentieren. Diese Rohdaten müssen vom Microcontroller empfangen und in physikalische Einheiten umgerechnet werden. Die Datenübertragung erfolgt in der Regel in mehreren Schritten:

1. **I2C-Initialisierung**: Der Microcontroller initialisiert die I2C-Schnittstelle und stellt eine Verbindung zur MPU-6050 her.
2. **Registerauswahl**: Der Microcontroller wählt die Register aus, die die gewünschten Messwerte enthalten.
3. **Datenübertragung**: Die Rohdaten werden aus den Registern der MPU-6050 ausgelesen und in den Speicher des Microcontrollers übertragen.

# Signal im Arduino Code nutzen

Um die Rohdaten der MPU-6050 in physikalische Einheiten umzuwandeln, müssen wir den Messbereich und die Auflösung des Sensors verstehen. Diese beiden Größen sind entscheidend für die Genauigkeit und den Anwendungsbereich der Messungen bzw von Sensoren im Allgemeinen.

**Auflösung**: Die Auflösung eines Sensors gibt an, wie fein die Messungen sind, also wie viele unterschiedliche Werte der Sensor innerhalb seines Messbereichs unterscheiden kann. 

**Messbereich**: Der Messbereich eines Sensors gibt an, welche maximalen Werte der Sensor messen kann. Bei einem Accelerometer wird der Messbereich häufig in der Einheit $g$ angegeben, wobei $g$ die Erdbeschleunigung (ca. $9,81\ m/s²$) darstellt. Ein größerer Messbereich ermöglicht es, höhere Beschleunigungen zu messen, allerdings ist die Genauigkeit dann geringer, da ein größerer Bereich, bei der gleichen Auflösung, größere Sprünge zwischen den einzelen Schritten hat. 

Es muss also immer überlegt werden, wie groß man den Messbereich wählt. Ein zu großer Messbereich liefert ungenauere Messungen und somit auch eine schlechtere Regelung, jedoch kann es bei einem zu kleinen Messbereich passieren, dass ein Wert nicht gemessen werden kann. *Bei komplexeren Systemen verwendet man häufig mehrere Sensoren, um sowohl einen großen Messbereich als auch eine gute Auflösung zu erzielen.* Bei unserem Roboter arbeiten wir aber nur mit einem Sensor 

![image](https://github.com/user-attachments/assets/92eed114-74f8-4403-94b4-f2abc41a769f)


Oben: größerer Messbereich, dafür schlechtere Genauigkeit, da mehr Abstand zwischen den Punkten

Unten: geringerer Messbereich, dafür genauere Messung möglich



Die von uns verwendete MPU-6050 bietet verschiedene Messbereiche, die über die Konfiguration des Sensors eingestellt werden können. Die Messbereiche sind ±2g, ±4g, ±8g und ±16g. Die Auflösung ist fest und beträgt 16 bit.

**wie viele Zahlen können mit 16 bit dargestellt werden?** 2^16


Wir verwenden den Messbereich +-2g, **Begründung...** 
Wie kann man die Rohdaten auf physikalische Einheiten in $m/2$  umrechnen? **Faktor...**

Umrechnungsfaktor=655364g​=655364×9.81m/s2​≈0.0006m/s2 pro LSB

Da die MPU-6050 oft mit einem Umrechnungsfaktor von 16384 für den ±2g-Bereich angegeben wird, bedeutet dies, dass:

1LSB=163842g​=163842×9.81m/s2​≈0.0012m/s2
