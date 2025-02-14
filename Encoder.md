# Einleitung
<br>
Ein Motor kann die Drehzahl eines angeschlossenen Rades nicht direkt einstellen, sondern lediglich die anliegende Spannung variieren. Da die tatsächliche Radgeschwindigkeit jedoch auch vom Widerstand (z. B. durch Gelände oder Last) abhängt, ist es notwendig, ein Feedback-System zu verwenden, um die Drehzahl präzise zu regeln. Hier kommt der magnetische Quadratur-Encoder ins Spiel. Mithilfe des Encoders kann der Motor die tatsächliche Drehzahl erfassen und die Spannung entsprechend anpassen, um die gewünschte Geschwindigkeit zu erreichen.

Ein Quadratur-Encoder arbeitet, indem er ein Signal erzeugt, wenn sich die Achse des Motors oder des Rades um einen bestimmten Winkel gedreht hat. Der von uns verwendete Encoder kann nicht nur die Drehbewegung erfassen, sondern auch zwischen Vorwärts- und Rückwärtsbewegung unterscheiden. Dadurch ermöglicht er eine präzise Kontrolle der Drehrichtung und der Geschwindigkeit.

Der Encoder basiert im Wesentlichen aus einer kleinen Magnetscheibe, die auf der Achse montiert ist und einem Zwei-Kanal-Hall-Effekt-Sensor. Wenn sich die Scheibe dreht, wird ihre Bewegung von dem Sensor erfasst. Der Hall-Effekt-Sensor reagiert auf die Änderung des Magnetfelds, das von der rotierenden Magnetscheibe erzeugt wird. Pro vollständiger Umdrehung der Magnetscheibe liefert der Sensor 12 Impulse (Counts). Diese Impulse entsprechen den Winkeländerungen der Achse und können genutzt werden, um die Drehgeschwindigkeit und Drehrichtung zu berechnen.

Wie diese Impulse in die tatsächliche Winkeländerung des Rades umgerechnet werden, hängt von der Radgeometrie und der Übersetzung ab. Diese Berechnung wird in dem Test behandelt.
<br>
# Funktionsweise des Sensors
<br>

## Prinzip 
<br>
Der Encoder kann die Drehrichtung der Achse unterscheiden, indem er die relative Phasenverschiebung zwischen den Signalen seiner beiden Hall-Sensoren auswertet. Dafür sind die zwei Hall-Effekt-Sensoren auf der Drehachse um 90° zueinander versetzt angeordnet. Wenn die Achse, und somit die Magnetscheibe, rotiert, erfassen die Hall-Effekt-Sensoren die abwechselnden Magnetpole der Scheibe und senden, je nach Polarität, ein HIGH oder LOW Signal. Aufgrund der Anordnung der beiden Sensoren werden die Magnetpole zeitlich versetzt von den Sensoren erkannt. Durch diese zeitliche Differenz kann die Drehrichtung bestimmt werden.

![Pasted image 20250213151345](https://i.imgur.com/ECEkNiG.png)

## Flankenerkennung

Jeder der beiden Sensoren erzeugt ein digitales Rechtecksignal, das zwischen HIGH (z. B. Nordpol) und LOW (z. B. Südpol) wechselt, während sich die Magnetscheibe dreht. Die Elektronik, die das Signal auswertet, erkennt dabei die Flanken der Signale:

- **Steigende Flanke:** Der Wechsel von LOW zu HIGH.
- **Fallende Flanke:** Der Wechsel von HIGH zu LOW.

Durch den Vergleich der Flanken des ersten und zweiten Signals kann die Drehrichtung bestimmt werden:

- Wenn das Signal des ersten Sensors (A) der steigenden Flanke des zweiten Signals (B) vorausgeht, dreht sich die Magnetscheibe in die eine Richtung (z. B. vorwärts).
- Wenn das Signal des ersten Sensors (A) der steigenden Flanke des zweiten Signals (B) nachläuft, dreht sich die Magnetscheibe in die entgegengesetzte Richtung (z. B. rückwärts).

## Quadratur-Signale

Die beiden phasenverschobenen Signale (A und B) werden als Quadratur-Signale bezeichnet. Diese Phasenverschiebung stellt sicher, dass die Reihenfolge der Flanken eindeutig ist. Es gibt insgesamt vier Möglichkeiten, die in einem festen Muster durchlaufen werden:

1. A = LOW, B = LOW
2. A = HIGH, B = LOW
3. A = HIGH, B = HIGH
4. A = LOW, B = HIGH

Die Reihenfolge, in der diese Zustände auftreten, bestimmt die Drehrichtung. In einer Richtung werden die Zustände beispielsweise als 1 → 2 → 3 → 4 durchlaufen, in der anderen Richtung als 4 → 3 → 2 → 1. Diese Reihenfolge muss jedoch nicht komplett durchlaufen werden, die Drehrichtung kann sich auch vorher ändern. So wird zum Beispiel bei der Reihenfolge 4 → 3 → 4 → 1 erkannt, dass die Drehrichtung sich geändert hat.

Diese vier Zustandsänderungen machen deutlich, warum die Sensoren um 90° und nicht um 180° zueinander versetzt angeordnet sind. Bei einem Versatz von 180° würden die Signale immer genau invers zueinander sein, worduch man nicht unterscheiden könnte, welches Signal vorausläuft und welches nachläuft. Die Flankenwechsel beider Signale würden immer gleichzeitig auftreten – nur mit entgegengesetzten Pegeln. 

![Pasted image 20250213171948](https://i.imgur.com/yplbG7H.png)
Signale der Sensoren im Oszilloskop: Es ist zu erkennen, dass das grüne Signal dem gelben vorgeht, da so der zeitliche Abstand zwischen den jeweiligen Flankenänderungen geringer ist.

