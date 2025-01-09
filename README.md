
# Übersicht

- [x] [[#Einleitung]]
- [x] [[#Arduino Programmcode]]
	- [x] Aufbau eines Arduino-Sketchs incl Abwandlung des Loops
	- [x] Präprozessor-Direktiven
- [ ] [[#Serielle Kommunikation]]
	- [x] was + warum
	- [x] Verbindung aufbauen
	- [x] Serial.print() und Serial Monitor
	- [ ] Serial.Read()
	- [x] Übungsaufgaben
- [x] [[#Input und Output]]
	- [x] Pins vom MCU vorstellen
	- [x] Pinmodus setzen
	- [x] digital vs analog
	- [x] Eingänge auslesen
		- [x] digital
		- [x] analog
	- [x] Ausgänge setzen
		- [x] digital
		- [x] PWM
	- [x] Interrupts
- [ ] [[#Variablen]]
	- [x] Vorstellung
	- [x] Datentypen
	- [x] Deklaration, Wertzuweisung, Initialisierung
	- [ ] Keywords
		- [x] const, volatile, static
		- [ ] ?Gültigkeisbreich, public, private?
	- [x] Übungsaufgaben
- [x] [[#Verzweigungen - if]]
	- [x] Vorstellung
	- [x] Übungsaufgaben
- [x] [[#Schleifen]]
	- [x] while + do while
		- [x] Vorstellung while
		- [x] Übungsaufgaben while
		- [x] Vostellung do while
		- [x] Übungsaufgaben do while
	- [x] for
		- [x] Vorstellung
		- [x] Übungsaufgaben
- [x] [[#Ausdrücke]]
- [x] [[#Funktionen]]
	- [x] Vorstellung
	- [x] Funktion erstellen und nutzen
	- [x] einzelne wichtige Funktionen vorstellen (delay, millis, micros, mathezeugs)
	- [x] Übungsaufgaben
	- [x] Bibliotheken
- [x] [[#Arrays]]
	- [x] Vorstellung
	- [x] Arrays erstellen
	- [x] Arrays verwenden



~={n}Allgemeines:
- [ ] casting (double zB) erklären
- [ ] erklären was passiert wenn man zb int x = 77.2727273 schreibt
- [x] i++ i+= usw erklären, vmtl bei Variablen - Wertzuweisungen?
	- [x] dabei auch auf ++i und i++ eingehen?
- [ ] nested loops brauchen wir nicht oder?
- [ ] für ausdrücke übungen zu && bzw ||?
- [ ] wie die hardware vorstellen=~



~={red}TODO:=~
- [ ] ...
- [ ] Reihenfolge prüfen, in welcher die Infos gegeben werden


# Einleitung 

die reine Verbindung der Komponenten reicht natürlich nicht aus, um ein
mechatronisches System betriebsbereit zu machen
- Welche Pins sind Eingänge? Welche sind Ausgänge?
- Wie sollen die Signale an den Pins interpretiert werden?
- Wie werden Daten von der IMU angefordert?
- Wie sind diese Daten aufgebaut? Wie sollen sie interpretiert werden?
- Wie soll das System auf die Messdaten reagieren?
- …
→ Mikrocontroller muss programmiert werden


wenn Effizienz des Programms nicht absolute Priorität hat, wird zur Programmierung in
der Regel eine „Hochsprache“ genutzt
- verwendet nicht unmittelbar den Befehlssatz der CPU
- Interpreter / Compiler übersetzt Anweisungen in Maschinensprache
- weniger abstrakt als maschinennahe Sprache und daher besser von Menschen lesbar
- besser zur Umsetzung komplexer Programme geeignet
- Beispiele: C, C++, C#, Java, Python, …

weit verbreitet ist die Nutzung von C und C++ zur Programmierung von MC
- bietet viele Annehmlichkeiten, die zur Gestaltung von übersichtlichem und damit gut wartbarem Code nützlich sind
	- Funktionen
	- Arrays
	- Klassen und Objekte
	- …
- Syntax ist ähnlich zu Java und C# (da deren Syntax an C angelegt ist)

Arduino-Plattform setzt bei der Programmierung auf C++, wobei viele Komplikationen der
Mikrocontroller-Programmierung von der Arduino-Plattform versteckt werden


# Arduino Programmcode

## Aufbau eines Arduino-Sketch

Ein Arduino-Sketch lässt sich überlicherweise in 3 Programmteile gliedern:

**globaler Bereich:**
- wird einmal duchlaufen
- Einbinden von Bibliotheken
- Definition von Konstanten / Symbolen
- Anlegen von globalen Variablen und Objekten

**setup Funktion:**
- wird bei jedem Neustart der MCU einmal durchlaufen
- Setzen von Pinmodi
- Initialisierung von Kommunikationsschnittstellen
- Initialisierung und Konfigurierung von Peripherie (Sensoren, Motortreiber, …)

**loop Funktion:**
- wird nach dem Durchlaufen der setup Funktion aufgerufen und wiederholt sich durchgehend
- beinhaltet den eigentlichen Programmablauf
- Lesen eines Sensorwerts über Eingangspins
- Schalten von Ausgangspins
- Prüfung von eingehenden Kommandos über eine der Schnittstellen (z.B. UART)


Wir werden jedoch auf die Verwendung der loop Funktion verzichten und stattdessen eine eigene Dauerschleife in der setup Funktion erstellen. ~={yellow}Dies entspricht auch dem üblichen Vorgehen bei anderen Programmierumgebungen. =~

```c++
void setup() {
	//Grundeinstellungen

	while(true) { //unsere eigene Dauerschleife
		//Programmcode
	}
}

void loop() {
	//diese Funktion bleibt bei uns leer
}
```

## Präprozessor-Direktiven

Anweisungen an den Compiler, welche vor der Kompilierung des Programms durchgeführt werden sollen
- Werden mit einer Raute # markiert
- Sie stehen idR. ganz oben über dem Rest des Codes

Für uns wichtige Präprozessor-Direktiven:
- \#include
	- Bindet Programmcode aus einer anderen Datei ein, sodass dieser verwendet werden kann
	- Die Include-Zeile wird quasi durch den Inhalt der hinzuzufügenden Datei ersetzt
	- \#include „filename.h“ durchsucht das lokale Verzeichnis nach der Datei
	- \#include <filename.h> durchsucht das Library-Verzeichnis nach der Datei.
- \#define
	- Ersetzt Text im Quellcode durch einen anderen Text
	- Beispiel:
		- \#define LED_PIN 5
		- Ersetzt überall im Quellcode den Ausdruck „LED_PIN“ durch das Literal „5“
		- Alternative zur Nutzung von Konstanten für die Bezeichnung der IO-Pins, die sich durchgesetzt hat



# Serielle Kommunikation

## Was ist serielle Kommunikation und wofür benötigen wir sie

Serielle Kommunikation ist eine Methode, um Daten zwischen dem Arduino-Board und anderen Geräten, wie Computern oder Sensoren, zu übertragen. Diese Kommunikation nutzt das UART-Protokoll (Universal Asynchronous Receiver/Transmitter) und erfolgt bitweise über eine serielle Verbindung, typischerweise über die USB-Schnittstelle oder die RX/TX-Pins. Sie wird verwendet, um Daten zu senden und zu empfangen, Debugging-Informationen an den Computer zu übertragen, oder um mit anderen Mikrocontrollern und Peripheriegeräten zu kommunizieren. Wir werden sie haupsächlich nutzen um Sensordaten und Systemzustände in Echtzeit zu überwachen.

**Serielle Kommunikation** überträgt Daten bitweise nacheinander über eine einzelne Leitung, was sie einfach und kostengünstig macht, aber langsamer als parallele Kommunikation. 

**Parallele Kommunikation** überträgt mehrere Bits gleichzeitig über mehrere Leitungen, was höhere Geschwindigkeiten ermöglicht, aber komplexer, fehleranfälliger und teurer ist. 

**Asynchrone Kommunikation** erfolgt ohne gemeinsamen Takt, wobei Sender und Empfänger ihre eigenen Taktgeber verwenden und Start- und Stoppbits zur Synchronisation nutzen. Dies ermöglicht flexible und robuste Datenübertragung, insbesondere in Umgebungen mit variablen Übertragungsraten. Alle Geräte müssen auf die gleiche Baudrate eingestellt sein, sie bezeichnet die Anzahl der Signaländerungen oder Symbole pro Sekunde. Die Baudrate bestimmt die Geschwindigkeit, mit der Daten übertragen werden, und wird in Baud (Bd) gemessen. Eine höhere Baudrate ermöglicht schnellere Datenübertragung, erfordert jedoch eine präzisere Synchronisation zwischen Sender und Empfänger.

- [x] ~={n}seriell, parallel, synchron, asynchon erklären?=~ 
~={red}hier kannst du Seriell, Parallel und Asynchron erklären, Synchron kommt dann bei I2C hinzu=~

## Verbindung aufbauen

Um die serielle Kommunikation aufzubauen und Daten auszutauschen muss zum einen natürlich die MCU mit dem PC per USB-Kabel verbunden sein und zum anderen muss der Programmcode entsprechende Befehle enthalten.

in void setup() muss der Befehl Serial.begin() ausgeführt werden. Dieser initialisiert die Serielle Verbindung (UART und USB-Serial) mit der angegebenen Baudrate.

Beispielaufruf: 
```c++
Serial.begin(9600);
```

- [x] ~={n}Baudrate erklären?=~
~={red}Nur sehr vereinfacht, also, dass es quasi die Übertragungsgeschwindigkeit ist und dies insbesondere für Asynchrone Übertragung wichtig ist, das bei die gleiche Baudrate haben=~
~={n}im Abschnitt oben erklärt=~

## Daten von MCU verschicken

Um die MCU Daten an den PC schicken zu lassen werden die Funktionen Serial.print() und Serial.println() verwendet. Beide Funktionen senden eine Zeichenfolge über die serielle Schnittstelle, wobei Serial.println() noch zusätzlich am Ende ein "Carriage-Return" und "Newline" Zeichen sendet, welche einem Zeilenumbruch entsprechen. 

Beispielaufruf: 
```c++
Serial.println("Hallo");
```

~={yellow} Wie Die Daten in der Arduino IDE angezeigt werden können wird, wird in der IDE Einführung erklärt.=~

- [x] ~={red}Hier einen Hinweis auf die IDE Einführung geben und den Teil weglassen. Wir sollten immer auf einer Ebene bleiben=~


## Serial.Read()

```c++
if (Serial.available() > 0) {
    // Lese die nächste Ganzzahl von der seriellen Schnittstelle
    int receivedNumber = Serial.parseInt();
    // Überprüfe, ob die Eingabe abgeschlossen ist
    // Achtet darauf, dass in der Seriellen Konsole nur NewLine ausgewählt ist
    if (Serial.read() == '\n') {
      // Übergib die Ganzzahl an die Funktion
      int16_t speedToSet[2] = { receivedNumber, receivedNumber };
      setSpeed(speedToSet);
}
```


## Übungsaufgaben

initialisiert eine serielle Kommunikation mit der Baudrate 115200.

Lösung:
```c++
void setup() {
	Serial.begin(115200);
}
```

gebt den Text "Hallo Welt!" aus.
```c++
Serial.println("Hallo Welt");
```

Gebt den Namen, den Wert und die Einheit (°) der Variable "Winkel" aus.
Beispielausgabe: `Winkel: 30°`

```c++
Serial.print("Winkel: ");
Serial.print(Winkel);
Serial.println("°");
```


# Input und Output

Die Pins des MCU können entweder als Input- oder Output-Pins betrieben werden, also um ~={yellow}Signale=~ einzulesen oder zu schreiben. Um den Pinmodus zu setzen wird die Funktion `pinMode(«Pin», «Mode»)` verwendet. `«Pin»` steht hierbei für die Nummer des Pin, welche gesetzt werden soll. Die Nummern können dem Pinout entnommen werden, wir benötigen die Nummer der GP Bezeichner. `«Mode»` gibt an, wie der Pin betrieben werden muss, zur Wahl stehen: INPUT, OUTPUT und INPUT_PULLUP. Die Modi INPUT und OUTPUT setzen den Pin entsprechend, INPUT_PULLUP setzt den Pin als input und nutzt dabei einen eingebauten pullup Widerstand. Dies ist für uns aber nicht von Bedeutung, da jeder relevante Pin über einen eigenen Widerstand auf der Platine verfügt.

![[pico-2-r4-pinout.svg]]


## digitale und analoge Signale

Im Kontext von Arduino sind digitale Signale binäre Signale, die nur zwei Zustände haben: HIGH (1) und LOW (0). Sie werden z.B. verwendet, um einfache Ein-/Aus-Schaltungen zu steuern, wie LEDs oder Relais, oder um Zustände von Schaltern zu erkennen. 
Analoge Signale hingegen sind kontinuierliche Signale, die eine Vielzahl von Werten innerhalb eines bestimmten Bereichs annehmen können. ~={yellow}Ein Microcontroller=~ kann analoge Signale lesen (z.B. von Sensoren) und verarbeiten, indem er sie in digitale Werte umwandelt (Analog-Digital-Wandlung). Diese Signale ermöglichen feinere Steuerungen und Messungen, wie die Helligkeit einer LED oder die Position eines Potentiometers. Arduino kann allerdings nicht analoge Pins schreiben. Um dennoch ein pseudoanaloges Ausgangssignal einstellen zu können, lernen wir später die Pulsweitenmodulation (PWM) kennen.


![[51c495ebce395f1b5a000000.png]]
digitales Signal

![[51c48875ce395f745a000000.png]]
analoges Signal

![[51c85fbece395fbc03000000.png]]
analoges Signal nach ADC 


## Eingänge auslesen

### digitale Eingänge

**`digitalRead(«Pin»)`**

was ist digitalRead()?
- gibt den Zustand des angegeben digitalen Pins aus
- kann als Wert nur 0/false und 1/true haben
- Dient dem Erfassen von binären Zuständen, wie Schalterstellungen, Tasten usw.

warum digitalRead()?
- Ideal, wenn einfache Ein-/Aus-Zustände genügen
- Weniger anfällig für Rauschen und Störungen
- Sehr schnell, da nur der Zustand eines Pins abgefragt werden muss
- digitale Eingänge benötigen oft nur einfache Komponenten wie Pullup- oder Pulldown-Widerstände

Beispiel:
	![[Pasted image 20241220152812.png]]

### analoge Eingänge

**`analogRead(«Pin»)`**

was ist analogRead()?
- Liest analoge Spannungswerte von einem analog-fähigem Pin
- Verwendet einen Analog-Digital-Wandler (ADC)
- Gibt standardmäßig einen digitalen Wert ~={yellow}von 10 Bit=~ ~~zwischen 0 und 1023~~ für Spannungen von 0 V bis 3,3 V zurück

warum analogRead()?
- Ermöglicht die Umwandlung von kontinuierlichen analogen Signalen in digitale Werte
- Ideal für den Einsatz mit verschiedenen Sensoren, die analoge Ausgangssignale liefern

Beispiel:
	![[Pasted image 20241220153001.png]]


Aufgabe:
Welchen Wert gibt `analogRead(2)` zurück, wenn an Pin 2 eine Spannung von $2.5\ V$ liegt?

Antwort:
$2.5\ V/3.3\ V*1023=775$

## Ausgänge schreiben

### digitale Ausgänge

**`digitalWrite(«Pin», «Wert»)`**

was ist digitalWrite()?
- setzt den Zustand des angegeben Pins auf den übergeeben Wert
- kann als Wert nur 0 und 1 haben
- Dient zB zum setzen von Zuständen 

Beispiel
```c++
digitalWrite(5, HIGH);
delay(1000);
digitalWrite(5, 0);
```

### analoge Ausgänge

**`analogWrite(«Pin», «Wert»)`**

Mithilfe der „Pulsweitenmodulation“ (PWM) lassen sich aus digitalen Signalen pseudoanaloge Signale erzeugen. An einem digitalen Output-Pin wird mit einer definierten Frequenz $f$ ein wechselndes Rechtecksignal angelegt. Der Mikrocontroller moduliert über die Pulsweite $g$  ~~(Verhältnis zwischen Länge des HIGH-Zustands und des LOW-Zustands)~~ ~={n}(Anteil des HIGH-Zustandes)=~ die Effektivspannung. Somit entspricht die Mittlere Spannung ~~dem Puls-Pause-Verhältnisses~~ ~={n}der Pulsweite=~ multipliziert mit der PIN-Spannung (3,3V)
$$\overline{U}=g*U_h$$
~={yellow}Standardmäßig hat die analogWrite() Funktion eine Auslögung von 8 Bits, diese Auflösung kann mit der Funktion `analogWriteResolution(«Wert»)` bei Bedarf geändert werden.=~

Aufgabe:
Setzt das PWM-Signal so, dass an Pin 6 eine mittlere Spannung von ca. 1 V anliegt.
_Hinweis: Die Logikspannung (HIGH-Spannung) des Raspberry Pi Pico W beträgt 3.3V und es wird die Standardauflösung von 8 Bits verwendet._

Lösung:
`analogWrite(6, 77);`
$x/255*3.3\ V=1$; 255 weil Wert von 0-255 gehen, dann $x=77.2727273$ runden auf $77$

- [x] ~={red}Hier fehlt noch der Hinweise zu den PWM-Bits=~ 
## Interrupts

Was sind Interrupts?
- Hardware- oder Software-Signale, die den normalen Ablauf des Programms unterbrechen ~={yellow}und dazu genutzt werden können sofort eine Interrupt-Funktion auszuführen=~
- Hardware-Interrupts: Ausgelöst durch externe Hardware-Signale (z.B. Tasten, Sensoren)
- Software-Interrupts: Ausgelöst durch Software-Signale (z.B. Timer)
- Die Interrupt-Funktionen sollten eine möglichst kurze Ausführungszeit haben ~={yellow}bspw. eine Variable ändern, jedoch keine andere Funktion ausführen=~
- Ermöglicht sofortige Reaktion auf ~~externe~~ Ereignisse

Warum Interrupts?
- Sofortige Bearbeitung von Ereignissen
- Keine Notwendigkeit für ständiges Polling (Überprüfen von Zuständen in einer Schleife)
- Mikrocontroller kann andere Aufgaben ausführen, bis ein Ereignis eintritt

`attachInterrupt(«Pin», «Funktionsname», «Änderungsart»)`

~={yellow}Für die Änderungsart gibt es 3 Möglichkeiten:
- **LOW:** Interrupt wird getriggert, wenn der Pin LOW ist
- **CHANGE:** Interrupt wird getriggert, wenn der Pin den Wert ändert
- **RISING:** Interrupt wird getriggert, wenn der Pin von LOW auf HIGH wechselt,
- **FALLING:** Interrupt wird getriggert, wenn der Pin von HIGH auf LOW wechselt.=~


Beispiel:
![[Pasted image 20241220154622.png]]

~={n}Arduino.cc empfiehlt `digitalPinToInterrupt(PIN)` zu verwenden, übernehmen? https://www.arduino.cc/reference/cs/language/functions/external-interrupts/attachinterrupt/=~



# Variablen

Variablen sind sozusagen temporäre Behälter für Daten, sie besitzen Bereiche im Speicher, in denen Daten abgelegt /gespeichert werden, um später darauf zugreifen zu können. Variablen haben einen Bezeichner (Namen), Datentyp, Wert und Speicherort.

**Bezeichner (Name):**
- Mit Hilfe des Namens der Variable kann auf den reservierten Speicherbereich bzw. den dort gespeicherten Wert zugegriffen werden.
- Regeln für Namensvergabe:
	- darf nicht mit Sonderzeichen oder Ziffern beginnen
	- üblicherweise mit kleinem Buchstaben anfangen
	- camelCase

**Datentypen und Werte:**  ([Q](https://www.kstbb.de/informatik/oo/07/7_1_Deklaration_von_Variablen.html))
- Ein Datentyp beschreibt eine Menge von Werten sowie die Operationen, die auf diese Werte angewandt werden dürfen.

für uns wichtige Datentypen:

| Datentyp | Bedeutung         | Speicherbedarf | Verwendung                                       |
| -------- | ----------------- | -------------- | ------------------------------------------------ |
| bool     | Wahrheitswert     | 8 Bit          | Systemzustände                                   |
| int      | Ganzzahl          | mind. 16 Bit   | Ganzzahlen von -32768 bis +32767                 |
| uint     | positive Ganzzahl | mind. 16 Bit   | Ganzzahlen von 0 bis 65536                       |
| long     | Ganzzahl          | 32 Bit         | Ganzzahlen von -2.147.483.648 bis +2.147.483.647 |
| float    | Gleitkommazahl    | 32 Bit         | Gleitkommazahl mit 6 Stellen                     |
| double   | Gleitkommazahl    | 64 Bit         | Gleitkommazahl mit 15 Stellen                    |
| char     | Zeichen           | mind. 8 Bit    | ASCII Zeichen (Ziffern, a-z, A-Z, Sonderzeichen) |
| string   | ??????            |                |                                                  |
unsigned verschiebt den abbildbaren Zahlenbereich ins positive -> doppelt so große Zahlen möglich, aber eben nur positive

Um keine bösen Überraschungen zu erleben, kann die Größe einiger Datentypen näher spezifiziert werden 

Beispiele:
int8_t → Ganzzahl der Länge 8 Bit
uint16_t → Ganzzahl der Länge 16 Bit

## Deklaration, Wertzuweisung, Initialisierung

### Deklaration ([Q](https://www.kstbb.de/informatik/oo/07/7_1_Deklaration_von_Variablen.html))

Bevor eine Variable verwendet werden kann, muss sie deklariert werden. Dabei werden ~={yellow}mindestens=~ ihr Name und ihr Datentyp festgesetzt. Stößt der Computer während der Laufzeit eines Programms auf eine Variablendeklaration, reserviert er für die Variable Speicherplatz in seinem Arbeitsspeicher. Wird der Variablen dann ein bestimmter Wert zugewiesen, wird dieser in dem für sie reservierten Speicherbereich abgelegt. Wie groß dieser Speicherbereich ist, hängt vom jeweiligen Typ der Variablen ab.

`«Datentyp» «Variablenname»;`

Beispiel:
```c++
bool start;
```

Aufgabe:
Deklariert eine Variable mit dem Namen "umdrehungen", ~={yellow}welche 0 bis 20000 Umdrehungen speichern kann und nur minimalen Speicherbedarf hat.=~

Lösung:
```c++
uint16_t umdrehungen;
```


### Wertzuweisung



##### Wertzuweisungen in C++

Bei einer Wert­zuweisung wird eine bereits deklarierten Variable mit einem tat­sächlichen Wert belegt. Die gebräuchlichste Form der Wertzuweisung erfolgt mit dem Gleichheitszeichen (`=`). 

Beispiel:
```c++
int a = 5; // Weist der Variablen a den Wert 5 zu
```


### Initialisierung

Wird einer Variablen zum ersten Mal ein Wert zugewiesen, wird dies als Initialisierung bezeichnet. Deklaration und Initialisierung können in einem Schritt erzeugen, eine Variable kann aber auch zu einem beliebigen späteren Zeitpunkt erst initialisiert werden.

Beispiel:
```c++
bool start = 0;

bool laeuft;
laeuft = 0;
```


### weitere Operatoren

Neben der einfachen Zuweisung gibt es auch zusammengesetzte Zuweisungsoperatoren, die eine Operation und eine Zuweisung in einem Schritt kombinieren. Diese Operatoren sind nützlich, um den Code kompakter und lesbarer zu machen. Zu den häufig verwendeten zusammengesetzten Zuweisungsoperatoren gehören:

- Additions-Zuweisungs-Operator `+=`: Addiert einen Wert zur Variablen und weist das Ergebnis der Variablen zu.
```c++
a += 3; // Entspricht a = a + 3; a ist jetzt 8
```


- Subtraktions-Zuweisungs-Operator `-=`: Subtrahiert einen Wert von der Variablen und weist das Ergebnis der Variablen zu.  
```c++
a -= 2; // Entspricht a = a - 2; a ist jetzt 6
```


- Multiplikations-Zuweisungs-Operator `*=`: Multipliziert die Variable mit einem Wert und weist das Ergebnis der Variablen zu.
```c++
a *= 2; // Entspricht a = a * 2; a ist jetzt 12
```


- Divisions-Zuweisungs-Operator `/=`: Dividiert die Variable durch einen Wert und weist das Ergebnis der Variablen zu.
```c++
a /= 3; // Entspricht a = a / 3; a ist jetzt 4
```


Weiterer wichtiger Operatoren sind der Inkrement-Operator (`++`) und der Dekrement-Operator (`--`), welche den Wert einer Variablen um eins erhöhen bzw verringern. Es gibt zwei Formen dieser Operatoren:

- Präfix-Operator (`++a`) bzw (`--a`): Erhöht bzw verringert den Wert der Variablen vor der Verwendung im Ausdruck.
```c++
++a; // Erhöht a um 1; a ist jetzt 6
```

- Postfix-Operator (`a++`): Erhöht bzw verringert den Wert der Variablen nach der Verwendung im Ausdruck.
```c++
a++; // Erhöht a um 1 nach der aktuellen Verwendung; a ist jetzt 5, danach 6
```


Aufgabe 1:
Weist der Variable "startWinkel" den Wert 25 zu.

Lösung:
```c++
a[startWinkel = 25;]
```


Aufgabe 2:
Deklariert und initialisiert eine Variable vom Datentyp int mit dem Namen "umdrehungen" und dem Wert 0, in einer einzigen Zeile.

Lösung:
```c++
a[int] b[umdrehungen] = c[0];
```


Aufgabe 3:
Erhöht den Wert der Variable "umdrehung" um 1.

Lösung:
```c++
a[umdrehung++];
```


Aufgabe 4:
Erhöht den Wert von i um 1 und lasst den erhöhten Wert ausgeben, es soll also `1` ausgegeben werden.

Lösung:
```c++
int i = 0;
Serial.print(a[i++])
```


Aufgabe 5:
Erhöhe den Wert von der Variable "winkel" um 15:
```c++
a[winkel] b[+=] c[15]
```


## Keywords

Keywords in C++ sind reservierte Wörter, die spezielle Bedeutungen im Sprachsyntax haben und nicht als Bezeichner für Variablen, Funktionen oder andere Benennungselemente verwendet werden können. Beispiele für Keywords sind `int`, `return`, `if`, `else`, `while`, `for`, `class`, und `public`. Sie definieren die Struktur und den Ablauf eines Programms, indem sie Datentypen, Kontrollstrukturen, Zugriffsmodifikatoren und andere grundlegende Sprachkonstrukte festlegen. 

Für uns wichtige Keywords:
**const:**
- Markiert Variablen nach ihrer Initialisierung als unveränderlich (konstant)
- Hilft dem Compiler bei der Optimierung und erhöht die Sicherheit des Codes, indem unbeabsichtigte Änderungen verhindert werden
- ~={red}Wo wird die Variable gespeichert? Insbesondere static const=~

**volatile:**
- ~={red}Hier fehlt noch was, volatile verhindert das Entfernen von Variablen, die der Compiler für ungenutzt hält, da sie nicht im normalen Programm aufgerufen werden=~
- Informiert den Compiler, dass eine Variable jederzeit von Prozessen außerhalb des normalen Programmflusses geändert werden kann (z. B. durch Hardware oder Interrupts)
- Der Compiler wird dadurch daran gehindert, Optimierungen vorzunehmen, die den Wert der Variable oder die Variable selbst entfernen könnten

**static:**
- Beeinflusst die Lebensdauer und Sichtbarkeit einer Variablen
- In einer Funktion deklarierte static-Variablen behalten ihren Wert zwischen den Funktionsaufrufen
- Diese Variablen werden nur beim ersten Aufruf der Funktion initialisiert, danach muss der Wert der Variable ohne das Keyword geändert werden

~={n}?Gültigkeitsbereich /scope und public /private /global?=~

~={red}Spielt nur eine Rolle, wenn wir objektorientierte Programmierung mit einbringen, oder?=~
~={n}und bei globalen Variablen, falls jmd auf die Idee kommt, seine Variable an der falschen Stelle zu deklarieren, aber vmtl durch Weglassen der loop Funkion egal=~


# Verzweigungen - if

Verzweigungen dienen dazu, ein Programm in mehrere Pfade aufzuteilen (z.B. um auf Eingaben des Benutzers zu reagieren). In C++ werden Verzweigungen mit dem Schlüsselwort `if` (Englisch für falls) begonnen. Optional kann eine Verzweigung mit dem  Schlüsselwort `else` (Englisch für ansonsten) erweitert werden. Die darauffolgenden Anweisungen werden ausgführt, wenn die ersten Bedingungen nicht erfüllt werden. Somit lassen sich beliebig komplexe Verzweigungen erstellen

simple if Verzweigung
```c++
if («Bedingung») {
	«Anweisungen»
}
«weiter im Programm»
```
![[Pasted image 20241220170718.png||500]]


komplexere if Verzweigung, beliebig erweiterbar
```c++
if («Bedingung 1») {
	«Anweisungen 1»
}
else if («Bedingung 2») {
	«Anweisungen 2»
}
else {
	«Anweisungen 3»
}
«Anweisungen 4»
```

![[Pasted image 20241220170531.png|500]]


## Übungsaufgaben 

**Aufgabe 1:**

Gebt über die serielle Schnittstelle den Text "x ist gleich y" aus, wenn x genau so groß ist wie y.

```c++
int x = 50;
int y = 50;
a[if] (x b[==] y) {
  Serial.print(c["x ist gleich y"]);
}
```


**Aufgabe 2:**

Gebt über die serialle Schnittstelle den Text 
- "Fall 1" aus, falls x kleiner als 100 ist
- "Fall 2" aus, falls $x \in [100;200]$
- "Fall 3" aus, falls x echt größer als 200 ist

```c++
int x = 201;

Serial.print(a["Fall "]);

i[if](x b[<] 100){
	Serial.println(c["1"]);
}
d[else if](x e[<=] 200){
	Serial.println(f["2"]);
}
g[else]{
	Serial.println(h["3"]);
}
```



# Schleifen

Schleifen ermöglichen, dass ein Programmteil mehrfach ausgeführt wird. 

Schleifen ermöglichen es, einen Codeblock wiederholt auszuführen, solange eine bestimmte Bedingung erfüllt ist. Die häufigsten Schleifenarten sind `for`, `while` und `do-while`. Schleifen sind essenziell für Aufgaben wie das Wiederholen von Berechnungen und das Verarbeiten von Datenmengen und das Durchlaufen von Arrays.


## (do) while-Schleife

Eine `while`-Schleife läuft, solange die Bedingung wahr ist, und eine `do-while`-Schleife garantiert mindestens eine Ausführung des Codeblocks, da die Bedingung erst am Ende geprüft wird. 

### while-Schleife
```c++
while («Bedingung») {
	«Anweisungen»
}
```
![[Pasted image 20241220175030.png|500]]


Aufgabe 0:
Wann wird eine while Schleife verwendet?

Lösung: wenn ein Codeblock mehrfach ausgeführt werden soll, so lange eine Bedingung erfüllt ist.


Aufgabe 1:
Gebt den wert der Variable i aus, falls ihr Wert kleiner ist als 6.
```c++
int i = 1;

 a[while](i < 6) {
	Serial.println(i);
	ib[++];
}
```


Aufgabe 2:
Was ist die Ausgabe des folgenden Codes?
```c++
int i = 0;

while(i < 3){
	Serial.print(i);
	i++;
}
```

Lösung: 012

Wie kann der Code verändert werden, damit die Ausgabe lautet: `0 1 2`?
```c++
int i = 0;

while(i < 3){
	Serial.print(i);
	Serial.print(" ");
	i++;
}
```


Aufgabe 3:
Was ist die Ausgabe des folgenden Codes?
```c++
int x = 5;

while(x > 0){
	Serial.print(x--);
	Serial.print(" ");
}
```

Lösung: 5 4 3 2 1


### do while-Schleife
```c++
do {
	«Anweisungen»
} while («Bedingung»);
```

![[Pasted image 20241220174947.png||500]]


Aufgabe 0:
Wann ist eine do while statt einer while Schleife sinnvoll?

Lösung: wenn der Codeblock immer mindestens 1 mal durchlaufen soll


Aufgabe 1:
Gebt den wert der Variable i aus, falls ihr Wert kleiner ist als 6.
```c++
int i = 1;

a[do]{
	Serial.print(i);
	b[i++];
}
 c[while](i < 6);
```


Aufgabe 2:
Was ist die Ausgabe des Codes?
```c++
int i = 3;

do{
	Serial.print(i);
	i++;
} while(i < 3);
```

Lösung: 3


Aufgabe 3:
Was passiert beim Ausführen des folgenden Codes?
```c++
int i = 5;
do{
	Serial.print(i)
} while(i > 0)
```

Lösung: es wird in einer Dauerschleife "5" ausgegeben, da i nicht verändert wurde und somit die Abbruchbedingung nie erfüllt werden kann.


## for-Schleife

Eine for-Schleife wird in der Regel genutzt, um Anweisungen mit einer definierten Zahl von Wiederholungen auszuführen. Sie besteht aus einem Initialisierungsteil, einem Bedingungsteil und einem Anweisungsteil. 

**Initialisierungsteil:**
- wird einmal zu Beginn ausgeführt
- wird in der Regel genutzt, um eine Zählvariable anzulegen

**Bedingungsteil:**
- äquivalent zur while- und do-while Schleife

**Anweisungsteil:**
- wird jedes mal nach Durchführung der Anweisungen im Schleifenrumpf ausgeführt
- wird idR. genutzt um die Zählvariable zu verändern

![[Pasted image 20241220181130.png|500]]


Aufgabe 0:
Wann zieht man die for Schleife einer While Schleife vor?

Lösung: Wenn man genau weiß, wie oft man die Schleife ausführen werden soll


Aufgabe 1:
Erzeugt mit Hilfe der for Schleife die Ausgabe `0 1 2 3 4`
```c++
 (int i a[=] 0; i a[<] 5; c[i++]) {
  Serial.print(i);
}
```



# Ausdrücke

Ausdrücke sind Kombinationen von Operanden und Operatoren, die zu einem Wert ausgewertet werden. Operanden können Variablen, Konstanten oder Funktionsaufrufe sein, während Operatoren mathematische (`+`, `-`, `*`, `/`), logische (`!`, `&&`, `||`) oder Vergleichsoperatoren (`==`, `!=`, `<`, `>`) umfassen. Ein Ausdruck wie `a + b` addiert beispielsweise die Werte von `a` und `b`. Ausdrücke sind grundlegende Elemente in Anweisungen und werden verwendet, um Berechnungen durchzuführen, Bedingungen zu prüfen und Werte zuzuweisen.

| wichtige Vergleichsoperatoren:           | wichtige Logikoperatoren:            |
| ---------------------------------------- | ------------------------------------ |
| ![[Pasted image 20241220171129.png]]<br> | ![[Pasted image 20250109231150.png]] |

Beispiele:
```c++
if(a != 10) //falls a nicht gleich 10

if(a > 0 && a < 10) //falls a größer 0 und a kleiner 10, also wenn a zwischen 0 und 10

while(a || b) //während mindestens a oder b wahr sind
```


Aufgabe 1:
Schreibe eine if-Verzweigung, welche erfüllt ist, wenn $a \in [-5;+5]$:
```c++
if(a > -5 && a < 5)
```


Aufgabe 2:
Schreibe eine while-Schleife, welche erfüllt ist, wenn a gleich 2 ist oder b ungleich 5:
```c++
while(a == 2 || b != 5)
```



# Funktionen

Eine Funktion ist ein Unterprogramm, das eine bestimmte Aufgabe erfüllt
- z.B. zur Kapselung sich wiederholender Aufgaben, sodass Befehle nicht jedes mal aufs neue geschrieben werden müssen
- verbessert die Übersichtlichkeit des Quellcodes
- Funktionen können einen(!) Rückgabewert haben und ihr können (mehrere) Parameter übergeben werden
- damit Funktionen ausgeführt werden, müssen sie aufgerufen werden, dies geschieht über den Baustein `«Funktionsname»(«Parameter»)`


## Funktionen deklarieren und aufrufen

- Funktionen müssen deklariert werden, bevor sie aufgerufen werden können. (Angabe des Funktionsprototyps)
- der Grundbaustein zum deklarieren einer Funktion sieht wie folgt aus:
	- `«Rückgabetyp» «Funktionsname»(«Parameter»)`

Beispiel:
```c++
int testFunktion(int x) //Funktionsdeklaration
```

```c++
testFunktion(5); //Funktionsaufruf
```


**Funktionen mit mehreren Parametern:**
Funktionen müssen keine Parameter besitzen, können aber beliebig viele besitzen
`«Rückgabetyp» «Funktionsname»(«Parameter1Datentyp» «Parameter1Name», «Parameter2Datentyp» «Parameter2Name»)`

Beispiel:
```c++
//Funktionsdeklarationen
int testFunktion2()
int testFunktion3(int x, int y)
```

```c++
 //Funktionsaufrufe
testFunktion2();
testFunktion3(1, 2);
```


**Funktionen ohne Rückgabe:**
wenn die Funktion keinen Rückgabewert haben soll, muss als Rückgabetyp das keyword `void` verwendet werden

`void «Funktionsname»(«Parameter»)`

Beispiel:
```c++
//Funktion ohne Rückgabe und ohne Parameter
void LED_Blink(){ //Funktionsdeklaration und Funktionsdefinition in einem
	digitalWrite(LED_PIN, HIGH);
	delay(250);
	digitalWrite(LED_PIN, LOW);
	delay(250);
}
```

```c++
//Funktion aufrufen
LED_Blink();
```


**Funktionen mit Rückgabe:**
- der Datentyp, welcher zurückgegeben werden soll, muss angegeben werden
- das Schlüsselwort zur Rückgabe des Funktionswert in der Definition lautet „return“
- der Rückgabewert kann einer Variable zugewiesen werden
- Innerhalb der Funktion erzeugte Werte und Variablen werden nach Funktionsaufruf zurückgesetzt bzw gelöscht
	- Sollen Variablen und deren Werte auch nach einem Funktionsaufruf erhalten bleiben,  enötigen sie das Keyword „static“ vor dem Datentypen
		- z. B. `static uint32_t counter;`

Beispiel:
```c++
//Funktion mit Rückgabe und 2 Parametern
int multipliziere(int x, int y); //Funktionsdeklaration
 
int multipliziere(int x, int y){ //Funktionsdefinition
	int z = x *y;
	return z;
}
```

```c++
//Funktion aufrufen
int mult = multipliziere(5, 9); //weist der Variable mult den Wert der Rückgabe zu
```


## wichtige Funktionen der Arduino Core-Library

`delay()`
Funktion: Pausiert das gesamte Programm für den angegebenen Zeitraum (in ms)
Beispielaufruf:  `delay(1000);`

`millis()`
Funktion: gibt die Zahl der Millisekunden, seit denen das Arduino-Programm läuft, wieder
Beispielaufruf: `value = millis();`

`micros()`
Funktion: gibt die Zahl der Mikrosekunden, seit denen das Arduino-Programm läuft, wieder
Beispielaufruf: `value = micros();`

`sqrt()`
Funktion: Berechnet die Wurzel einer Zahl
Beispielaufruf: `value = sqrt(9);`

`atan2()`
Funktion: Arcustangens (in Bogenmaß)
Beispielaufruf: `value = atan2(x,y);`


## Übungsaufgaben

Aufgabe 1:
Was ist eine Funktion?
- [ ] Eine Schleife die unendlich oft durchlaufen wird
- [ ] Ein Datentyp
- [ ] ein Pointer
- [x] Ein Codeblock, welcher nur durchlaufen wird, wenn er aufgerufen wird


Aufgabe 2:
Deklariert eine Funktion mit dem Datentyp "int", dem Funktionsnamen "addiere" und 2 Parametern, jeweils mit Datentyp int.

```c++
a[int] b[addiere](c[int] a, d[int] b)
```


Aufgabe 3:
Deklariert und definiert eine Funktion namens "hallo", welche keine Rückgabe und keine Parameter bestitz und über die bereits aufgabaute serielle Schnittstelle den Text "hallo Welt" schreibt. Ruft die Funktion in der Dauerschleife der setup Funktion auf.

```c++
void setup(){
	while(true){
		d[hallo();]
	}
}

a[void] b[hallo](e[]){
	c[Serial.println(d["hallo Welt"]);]
}
```


Aufgabe 4:
Deklariert und definiert eine Funktion namens "berechneWert" mit den folgenden Eigenschaften:
- Rückgabetypen double
- 3 Parameter, jeweils vom Typ double
- gibt die Summe von a und b geteilt durch c aus

 Weist der Variable "SensorWert" den Rückgabewert der Funktion mit den Parametern a =1, b=2 und c =3 zu.

```c++
void setup(){
	while(true){
		double sensorWert;
		h[sensorWert] = a[berechneWert(1, 2, 3);]
	}
}

b[double] c[berechneWert](d[double] a, e[double] b, f[double] c){
	g[return] (a +b) /c;
}
```


## Bibliotheken

Bibliotheken sind Sammlungen von vorgefertigten Funktionen, Klassen und Routinen. Sie bieten wiederverwendbare Lösungen für eine Vielzahl von Problemen und tragen zur Modularität und Wartbarkeit des Codes bei.

### Einbindung von Bibliotheken

Um eine Bibliothek in einem C+±Programm zu verwenden, muss sie zunächst eingebunden werden. Dies geschieht mit der `#include`-Direktive, die dem Compiler mitteilt, welche Header-Dateien (Dateien mit der Endung `.h` oder `.hpp`) in das Programm eingefügt werden sollen. Diese Header-Dateien enthalten die Deklarationen der Funktionen und Klassen, die in der Bibliothek definiert sind.

Beispiel:
```c++
#include <math.h>    // Einbindung der mathematischen Bibliothek
```


#### Verwendung von Bibliotheken

Nach der Einbindung der Bibliothek können die darin enthaltenen Funktionen und Klassen im Programm verwendet werden. Hier sind einige Beispiele:

**Ein- und Ausgabe mit der bereits bekannten Serial-Bibliothek**:
```c++
void setup() {
    Serial.begin(9600); // Initialisierung der seriellen Kommunikation mit 9600 Baud
    Serial.println("Hello, World!"); // Verwendung der println-Funktion aus der Serial-Bibliothek
}
```


**I2C-Kommunikation mit der Wire-Bibliothek**:
```c++
#include <Wire.h>

void setup() {
    Wire.begin(); // Initialisierung der I2C-Kommunikation
}
```



# Arrays

## Vorstellung

Arrays sind Datenstrukturen, die es ermöglichen, mehrere Werte desselben Datentyps unter einem gemeinsamen Namen zu speichern. Sie sind besonders nützlich, wenn man mit einer großen Anzahl von Daten arbeiten muss, wie z.B. Sensordaten, Messwerten oder einer Liste von Werten. Arrays erleichtern das Speichern, Verwalten und Verarbeiten von Daten, indem sie den Zugriff auf einzelne Elemente über Indizes ermöglichen.

## Arrays erstellen

In der Arduino IDE können Arrays durch die Deklaration des Datentyps, gefolgt vom Array-Namen und der Anzahl der Elemente in eckigen Klammern, erstellt werden. 
`«Datentyp» «Arrayname»[«Elementanzahl»];`

Beispiel:
```c+
double sensorWerte[5]; // Deklariert ein Array mit 5 double-Elementen
```


Ein Array kann direkt bei der Erstellung mit Werten gefüllt werden.
`«Datentyp» «Arrayname»[«Elementanzahl»] = {«Wert1», «Wert2»};`

Beispiel:
```c++
int sensorWerte[5] = {10, 20, 30, 40, 50}; //Initialisieren eines Arrays bei der Deklaration
```


### Arrays verwenden

Um auf einzelne Elemente eines Arrays zuzugreifen, muss man den entsprechenden Index angeben. Der Index beginnt bei 0 zu zählen, so hat das 1. Element eines Arrays also den Index 0.
`«Arrayname»[«Index»];`

Beispiel:
```
void setup() {
	Serial.begin(9600);
	int sensorWerte[5] = {10, 20, 30, 40, 50};
	
	Serial.println(sensorWerte[0]); // Gibt das erste Element des Arrays aus (10)
	Serial.println(sensorWerte[4]); // Gibt das fünfte Element des Arrays aus (50)
	
	int a = sensorWerte[2]; //Weist der Variablen a den Wert des dritten Array-Elements zu (30)
}
```


Werte von Array-Elementen können, ähnlich wie bei Variablen, durch Zuweisungen geändert werden.
`«Arrayname»[«Index»] = «Wert»;`

Beispiel:
```
void setup() {
	Serial.begin(9600);
	int sensorWerte[5] = {10, 20, 30, 40, 50};
	
	sensorWerte[2] = 35; // Ändert das dritte Element des Arrays auf 35
	Serial.println(sensorWerte[2]); // Gibt das geänderte Element des Arrays aus (35)
	
	sensorWerte[4] *= 2; // Ändert das fünfte Element des Arrays auf 100
	Serial.println(sensorWerte[4]); // Gibt das geänderte Element des Arrays aus (100)
}
```


Iteration durch Array-Elemente 
~={n}notwendig oder weglassen, evtl stattdessen als übungsaufgabe?=~
```
    void setup() {
        Serial.begin(9600);
        int sensorValues[5] = {10, 20, 30, 40, 50};
        for (int i = 0; i < 5; i++) {
            Serial.print("Element ");
            Serial.print(i);
            Serial.print(": ");
            Serial.println(sensorValues[i]); // Gibt jedes Element des Arrays aus
        }
    }
    
    void loop() {
        // Hauptprogrammcode
    }
```







------
# misc
https://www.youtube.com/playlist?list=PLGs0VKk2DiYw-L-RibttcvK-WBZm8WLEP
https://www.w3schools.com/cpp/cpp_variables.asp

## Speicher(bedarf)

CPU /Computer arbeiten mit ***binären Zahlen***  
***bits und bytes*** kurz erklären


## Variablen
Beispiele von Deklaration und Initialisierung, jeweils einzeln und zsm, markieren was Datentyp, Keyword, Name, und Inhalt ist

### Datentypen 

#### bool
speichert Wahrheitswert
wird genutzt um Systemzustände zu speichern /ändern


### int 
speichert Ganzzahlen 2^16


### float 
speichert Gleitkommazahlen 2^32
für uns am wichtigsten


### char und string 
aufgaben mit "" und '' als Auswahl



### Keywords 
#### const volatile static 


#### globale /lokale Variablen


### Deklaration und Initialisierung 


## Funktionen

### Ardiono eigene Funktionen

#### Zeit
##### delay() und millis()


#### Pins 
##### PinMode()


##### digital vs analog
erklärung


##### digital
###### digitalRead() und digitalWrite()


#### analog
##### analogRead() und (analogWrite() und PWM)


#### Interrupts




## Verzweigungen /Schleifen
ganz grob erklären, was die machen, schaubild mit Blöcken

### Vergleichsoperatoren
notwendig für Verzweigungen /Schleifen 
	Test ob x\==1 oder x=1 


### Verzweigung: if ?und case?


### Schleifen
#### for
(kann eigentlich auch while ablösen)

#### (do) while
?währenddessen keine anderen Operationen möglich -> in verbindung mit millis vermeiden?






# Aufgaben /Lektionen


## if
Bild von https://app.code2flow.com/ mit Code:

```
vorheriger CodeBlock

if(Bedingung){
  CodeblockWahr;
}
else{
  CodeblockFalsch
}
nächster Codeblock
```


## static

Welchen Wert hat i an der markierten Stelle, wenn man die Funktion drei mal ausführt?

```c++
void Add() {
  int i = 0;
  i++;
  //Marke
}
```

Lösung: 1
Begründung: i wird nach beenden der Funkion wieder gelöscht und beim nächsten Funktionsaufruf neu erstellt und auf 0 gesetzt. Die Marke liegt nach i++, also nach 0++ also 1


wie kann der Codeblock geändert werden, damit der Wert für i bestehen bleibt und somit nach drei Aufrufen i an der Marke den Wert 3 hat?

```c++
void Add() {
  static int i = 0;
  i++;
  //Marke
}
```

Erklärung von static
im setup als global erstellen würde auch gehen aber vermeiden globale variablen




gyro erklärung
6 achsen bild, welche davon sind relevant für den roboter?
ohne magnetometer, was ist das
später wire lib an








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

Beim Roboter kann dies aber auf 2 Achsen reduziert werden, da die dritte im normalen Betrieb immer senkrecht zum Gravitationsvektor steht und somit keinen Anteil des Gravitationsvektors aufnimmt.


![image](https://github.com/user-attachments/assets/37c935f7-4180-4ed1-b548-0607257865b6)
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
