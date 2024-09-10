# Kinect Gesture Control

Dit document beschrijft de installatie en het gebruik van de Kinect Gesture Control-software. Dit project gebruikt een Kinect-sensor om handbewegingen te tracken en deze te vertalen naar muisbewegingen en toetsenbordacties. Het systeem kan worden ingezet voor handsfree besturing van een computerscherm.

## Vereisten

Voordat je begint, zorg ervoor dat je aan de volgende vereisten voldoet:

- **Hardware**: Kinect Azure-sensor.
- **Software**:
  - Python 3.x
  - `opencv-python`
  - `pyautogui`
  - `pykinect-azure`
  - `numpy`
  - Een werkend Kinect Azure SDK
- **Besturingssysteem**: Windows 10 of nieuwer (met Kinect-support).
  
### Vereiste Python-bibliotheken

Voer het volgende commando uit om de vereiste Python-pakketten te installeren:

```bash
pip install opencv-python pyautogui pykinect-azure numpy
```

## Installatie en Setup

### 1. Kinect Azure SDK

Installeer eerst de Kinect Azure SDK volgens de officiële [Microsoft Kinect Azure-documentatie](https://learn.microsoft.com/en-us/azure/kinect-dk/).

### 2. Code Configuratie

Kopieer de code uit `main.py` naar je projectdirectory en pas indien nodig de instellingen aan zoals de Kinect-resolutie en FPS. Je kunt de configuratieopties instellen in het volgende gedeelte van de code:

```python
device_config = Configuration()
device_config.color_resolution = pykinect.K4A_COLOR_RESOLUTION_720P
device_config.depth_mode = pykinect.K4A_DEPTH_MODE_NFOV_UNBINNED
device_config.camera_fps = pykinect.K4A_FRAMES_PER_SECOND_30
```

### 3. Starten van het programma

Voer het volgende commando uit om het programma te starten:

```bash
python main.py
```

## Gebruik

Het systeem volgt handbewegingen om muisacties uit te voeren en toetsenbordinput te simuleren. Hier zijn de belangrijkste functies:

- **Muisbeweging**: De positie van de rechterhand wordt vertaald naar de muiscursor op het scherm.
- **Klikken**: Als de cursor langere tijd op dezelfde positie blijft, wordt een muisklik uitgevoerd.
- **Swipe bewegingen**: Snelle handbewegingen naar links, rechts, boven of onder worden gedetecteerd en vertaald naar de respectieve pijltjestoetsen.
- **Spatiebalk actie**: Als beide handen boven het hoofd worden gehouden, wordt de spatiebalk ingedrukt.

### Overige Functionaliteiten:

- **Hovering**: Wanneer de cursor boven een knop zweeft, wordt na een ingestelde tijd automatisch geklikt.
- **Visuele feedback**: Het Kinect-beeld toont live de gescande lichaamsdelen en bewegingen.

## Externe Bibliotheken

Het project maakt gebruik van de volgende externe bibliotheken:

- **OpenCV**: Voor het verwerken en tonen van beelden.
- **PyAutoGUI**: Voor het simuleren van muis- en toetsenbordacties.
- **Pykinect-Azure**: Voor Kinect-integratie en het tracken van lichaamspunten.
- **Numpy**: Voor numerieke berekeningen.

## Licentie

Dit project valt onder de BSD-licentie. Zie het [LICENSE](LICENSE) bestand voor de volledige licentievoorwaarden.

## Bijdragen

Bijdragen aan dit project zijn welkom. Gelieve bij te dragen volgens de geldende licentievoorwaarden en zorg ervoor dat je wijzigingen goed documenteert.

## Disclaimer

Dit project wordt geleverd "zoals het is" zonder enige garantie. Zie het [LICENSE](LICENSE) bestand voor meer details.
