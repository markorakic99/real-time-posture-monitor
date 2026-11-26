/*
 * DZ2 - Pametni monitor za drzanje kicme (Arduino deo)
 * Praktikum iz merno-akvizicionih sistema (PMS)
 *
 * Uloga Arduino ploce je ogranicena na komunikaciju sa senzorima i
 * indikatorima: akvizicija akcelerometra na svakih 100 ms (TimerOne
 * biblioteka), detekcija dodira kapacitivnog senzora (spoljni prekid
 * na D2), prikaz broja prekrsaja na TM1637 sedmosegmentnom displeju i
 * upravljanje ugradjenom LED diodom upozorenja. Sva obrada signala
 * (kalibracija, racunanje ugla, brojanje prekrsaja, tajmovanje) vrsi
 * se na Python strani - arduino samo prosledjuje sirove podatke i
 * izvrsava primljene komande.
 */

#include <TimerOne.h>
#include <TM1637Display.h>

// Raspored pinova prati tabelu iz teksta zadatka (TM1637, kapacitivni
// dodirni senzor, akcelerometar na analognim ulazima)
const uint8_t PIN_CLK = 9;
const uint8_t PIN_DIO = 8;
const uint8_t PIN_TOUCH = 2;
const uint8_t PIN_LED = LED_BUILTIN;  // ugradjena LED na pin 13
const uint8_t PIN_AX = A0;
const uint8_t PIN_AY = A1;
const uint8_t PIN_AZ = A2;

const unsigned long SAMPLE_PERIOD_US = 100000UL;  // 100 ms, zahtev iz zadatka
const unsigned long TOUCH_DEBOUNCE_MS = 300UL;

TM1637Display display(PIN_CLK, PIN_DIO);

// Ove tri promenljive se menjaju iz prekidnih rutina, pa moraju
// biti volatile - inace kompajler moze da zapamti staru vrednost i
// ne primeti izmenu koja se desila van normalnog toka programa.
volatile bool sampleReady = false;
volatile bool touchDetected = false;
volatile unsigned long lastTouchMillis = 0;

bool acquisitionActive = false;  // da li Arduino trenutno treba da salje odbirke
String rxLine;                  // bafer za sastavljanje komande karakter po karakter

void onTimerTick() {
  sampleReady = true;
}

// ISR za dodirni senzor (spoljni prekid na D2). Debounce od 300ms
// sprecava da jedan fizicki dodir (koji elektricno "odskace") bude
// registrovan kao vise uzastopnih dodira.
void onTouchInterrupt() {
  unsigned long now = millis();
  if (now - lastTouchMillis > TOUCH_DEBOUNCE_MS) {
    touchDetected = true;
    lastTouchMillis = now;
  }
}

void setup() {
  Serial.begin(115200);
  rxLine.reserve(16);

  pinMode(PIN_TOUCH, INPUT);
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  display.setBrightness(0x0f);
  display.showNumberDec(0);

  // Detekcija dodira preko prekida, kako trazi tekst zadatka
  attachInterrupt(digitalPinToInterrupt(PIN_TOUCH), onTouchInterrupt, RISING);

  // TimerOne konfigurisan na tacno 100ms period akvizicije, ali se ne
  // pokrece odmah - ceka se komanda START sa PC-ja
  Timer1.initialize(SAMPLE_PERIOD_US);
  Timer1.attachInterrupt(onTimerTick);
  Timer1.stop();
}

void loop() {
  handleIncomingCommands();

  // Prijavi svaki detektovani dodir Python strani, koja odlucuje sta
  // znaci u datom trenutku (pocetak kalibracije, pocetak testa, ...)
  if (touchDetected) {
    touchDetected = false;
    Serial.println("T");
  }

  // Salji odbirak akcelerometra samo kad je akvizicija ukljucena I kad
  // je tajmer upravo otkucao (sprecava slanje brze/sporije od 100ms)
  if (acquisitionActive && sampleReady) {
    sampleReady = false;
    int ax = analogRead(PIN_AX);
    int ay = analogRead(PIN_AY);
    int az = analogRead(PIN_AZ);
    Serial.print("A,");
    Serial.print(ax);
    Serial.print(",");
    Serial.print(ay);
    Serial.print(",");
    Serial.println(az);
  }
}

void handleIncomingCommands() {
  const uint8_t MAX_LINE_LEN = 32;  // zastita RAM-a od neogranicenog rasta (npr. sum na liniji bez '\n')
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n') {
      processCommand(rxLine);
      rxLine = "";
    } else if (c != '\r') {
      if (rxLine.length() >= MAX_LINE_LEN) {
        rxLine = "";  // neispravna/predugacka linija, odbaci i pocni ispocetka
      }
      rxLine += c;
    }
  }
}

// Prepoznaje i izvrsava komande koje salje Python strana (protokol
// opisan na vrhu fajla). Nepoznata komanda se tiho ignorise.
void processCommand(const String &cmd) {
  if (cmd == "START") {
    acquisitionActive = true;
    Timer1.restart();
  } else if (cmd == "STOP") {
    acquisitionActive = false;
    Timer1.stop();
    digitalWrite(PIN_LED, LOW);
    display.showNumberDec(0);
  } else if (cmd.startsWith("CNT,")) {
    int count = cmd.substring(4).toInt();
    display.showNumberDec(count);
  } else if (cmd == "LED,1") {
    digitalWrite(PIN_LED, HIGH);
  } else if (cmd == "LED,0") {
    digitalWrite(PIN_LED, LOW);
  }
}
