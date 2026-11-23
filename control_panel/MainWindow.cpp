#include "MainWindow.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QHeaderView>
#include <QPushButton>
#include <QFileDialog>
#include <QFileInfo>
#include <QFile>
#include <QMenuBar>
#include <QMenu>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QNetworkDatagram>
#include <QNetworkInterface>
#include <QShortcut>
#include <QRegularExpression>
#include <QtEndian>
#include <cmath>
#include <algorithm>

static constexpr double kPi = 3.14159265358979323846;

static QString portHint(const QString &proto, quint16 port) {
    return QString("   Sprawdź, kto trzyma port: netstat -ano | findstr :%1\n"
                   "   Jeśli nikt, port jest zarezerwowany przez Windows (Hyper-V/WSL/Docker): "
                   "netsh interface ipv4 show excludedportrange protocol=%2").arg(port).arg(proto);
}

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent),
      arucoDetector(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50),
                    []() {
                        cv::aruco::DetectorParameters p;
                        p.cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
                        p.cornerRefinementWinSize = 5;
                        p.minMarkerPerimeterRate = 0.06;
                        p.errorCorrectionRate = 0.35;
                        return p;
                    }()) {

    setWindowTitle("PicoGo Control Panel - IDM & ArUco Flocking");
    resize(1340, 760);

    commandSeq = static_cast<quint32>(QDateTime::currentSecsSinceEpoch() & 0x3fffffff);

    loadRobotDatabase();
    loadArenaConfig();
    setupMenuBar();
    setupUi();

    // 1. Gniazdo nasłuchujące meldunków HELLO (Port UDP 5006)
    udpListenSocket = new QUdpSocket(this);
    if (udpListenSocket->bind(QHostAddress::AnyIPv4, UDP_LISTEN_PORT, QUdpSocket::ShareAddress)) {
        connect(udpListenSocket, &QUdpSocket::readyRead, this, &MainWindow::onUdpReadyRead);
        logMessage(QString("Nasłuch telemetrii UDP uruchomiony na porcie %1.").arg(UDP_LISTEN_PORT));
    } else {
        logMessage(QString("BŁĄD: Nie można otworzyć portu UDP %1 (%2) – telemetria robotów nie będzie widoczna.")
                       .arg(UDP_LISTEN_PORT).arg(udpListenSocket->errorString()));
        logMessage(portHint("udp", UDP_LISTEN_PORT));
    }

    // 2. Gniazdo wysyłające rozkazy (Port UDP 5005)
    udpCommandSocket = new QUdpSocket(this);

    // 3. Serwer HTTP dla aktualizacji OTA (Port TCP 8000)
    httpOtaServer = new QTcpServer(this);
    if (httpOtaServer->listen(QHostAddress::AnyIPv4, HTTP_OTA_PORT)) {
        connect(httpOtaServer, &QTcpServer::newConnection, this, &MainWindow::onNewHttpConnection);
        logMessage(QString("Serwer HTTP OTA uruchomiony na porcie TCP %1.").arg(HTTP_OTA_PORT));
    } else {
        logMessage(QString("BŁĄD: Nie można otworzyć portu TCP %1 (%2) – aktualizacje OTA nie będą działać.")
                       .arg(HTTP_OTA_PORT).arg(httpOtaServer->errorString()));
        logMessage(portHint("tcp", HTTP_OTA_PORT));
    }

    // 4. Wbudowany serwer DHCP w tle (Port UDP 67)
    udpDhcpSocket = new QUdpSocket(this);
    connect(udpDhcpSocket, &QUdpSocket::readyRead, this, &MainWindow::onDhcpReadyRead);
    toggleDhcpServer(actDhcpEnable->isChecked());

    // 5. Timer odświeżający statusy ONLINE / OFFLINE co 500 ms
    statusTimer = new QTimer(this);
    connect(statusTimer, &QTimer::timeout, this, &MainWindow::refreshTableStatus);
    statusTimer->start(500);

    // 6. Timer klatek kamery (30 FPS -> ~33 ms)
    cameraTimer = new QTimer(this);
    connect(cameraTimer, &QTimer::timeout, this, &MainWindow::processCameraFrame);

    // Skrót klawiszowy: Spacja = Natychmiastowy STOP wszystkich robotów
    auto *spaceStop = new QShortcut(QKeySequence(Qt::Key_Space), this);
    connect(spaceStop, &QShortcut::activated, this, [this]() {
        stopVisionAll();
        sendStopCommand();
    });

    refreshCameras();
    rebuildVisionTable();
}

MainWindow::~MainWindow() {
    stopVisionAll();
    if (cameraCap.isOpened())
        cameraCap.release();
}

void MainWindow::setupMenuBar() {
    QMenu *netMenu = menuBar()->addMenu("⚙ Sieć i DHCP");

    actDhcpEnable = new QAction("Włącz wbudowany serwer DHCP w tle (Pula " + SUBNET_PREFIX + "21 - .250)", this);
    actDhcpEnable->setCheckable(true);
    actDhcpEnable->setChecked(true);
    connect(actDhcpEnable, &QAction::toggled, this, &MainWindow::toggleDhcpServer);
    netMenu->addAction(actDhcpEnable);
}

void MainWindow::setupUi() {
    auto *centralWidget = new QWidget(this);
    setCentralWidget(centralWidget);
    auto *rootLayout = new QVBoxLayout(centralWidget);
    rootLayout->setContentsMargins(6, 6, 6, 6);
    rootLayout->setSpacing(6);

    fleetSummaryLabel = new QLabel("Widoczne roboty: 0 | Online: 0 | Zaznaczone: 0 | DHCP: 0");
    fleetSummaryLabel->setStyleSheet("font-size: 13px; font-weight: bold; color: #4fc1ff;");
    rootLayout->addWidget(fleetSummaryLabel);

    mainTabs = new QTabWidget(this);
    mainTabs->setStyleSheet("QTabBar::tab { height: 32px; font-weight: bold; padding: 0 18px; }");
    mainTabs->addTab(createIdmTab(), "🚗 Symulacja 1: Tor IDM, PID & Wgrywanie OTA");
    mainTabs->addTab(createVisionTab(), "📷 Symulacja 2: Kamera ArUco, Koło & Ławica ryb");
    rootLayout->addWidget(mainTabs, 1);
}

QWidget* MainWindow::createIdmTab() {
    auto *tab = new QWidget(this);
    auto *mainLayout = new QHBoxLayout(tab);
    mainLayout->setContentsMargins(6, 6, 6, 6);
    mainLayout->setSpacing(10);

    // ==================== LEWA KOLUMNA: TABELA FLOTY ====================
    auto *leftLayout = new QVBoxLayout();
    leftLayout->setSpacing(6);

    fleetTable = new QTableWidget(0, 10, this);
    fleetTable->setHorizontalHeaderLabels({
        " Wybór ", "Nazwa (Edytuj)", "Adres IP", "Wersja", "Status",
        "Tryb", "Odległość [cm]", "Prędkość", "Freespeed", "Acc / Brake"
    });
    fleetTable->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    fleetTable->horizontalHeader()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
    fleetTable->setSelectionBehavior(QAbstractItemView::SelectRows);
    fleetTable->setAlternatingRowColors(true);
    connect(fleetTable, &QTableWidget::itemChanged, this, &MainWindow::onTableItemChanged);
    leftLayout->addWidget(fleetTable);

    auto *selBtnLayout = new QHBoxLayout();
    auto *btnSelAll = new QPushButton("☑ Zaznacz wszystkie");
    auto *btnDeselAll = new QPushButton("☐ Odznacz wszystkie");
    auto *btnSelOnline = new QPushButton("🟢 Zaznacz tylko ONLINE");
    connect(btnSelAll, &QPushButton::clicked, this, &MainWindow::selectAllRobots);
    connect(btnDeselAll, &QPushButton::clicked, this, &MainWindow::deselectAllRobots);
    connect(btnSelOnline, &QPushButton::clicked, this, &MainWindow::selectOnlineRobots);
    selBtnLayout->addWidget(btnSelAll);
    selBtnLayout->addWidget(btnDeselAll);
    selBtnLayout->addWidget(btnSelOnline);
    leftLayout->addLayout(selBtnLayout);

    auto *logGroup = new QGroupBox("Logi komunikacji, DHCP i zdarzeń");
    auto *logLayout = new QVBoxLayout(logGroup);
    logLayout->setContentsMargins(6, 6, 6, 6);
    logConsole = new QPlainTextEdit();
    logConsole->setReadOnly(true);
    logConsole->setMaximumBlockCount(500);
    logConsole->setStyleSheet("font-family: Consolas, monospace; font-size: 11px;");
    logLayout->addWidget(logConsole);
    leftLayout->addWidget(logGroup);
    leftLayout->setStretch(0, 3);
    leftLayout->setStretch(2, 1);

    // ==================== PRAWA KOLUMNA: PANELE STEROWANIA ====================
    auto *rightLayout = new QVBoxLayout();
    rightLayout->setSpacing(6);

    // 1. Szybkie akcje (bez osobnego "Identyfikuj zaznaczone", bączek -> Identyfikuj)
    auto *actionGroup = new QGroupBox("Sterowanie ruchem i kalibracja (Zaznaczone roboty)");
    auto *actionGrid = new QGridLayout(actionGroup);
    actionGrid->setContentsMargins(8, 6, 8, 6);

    auto *btnStart = new QPushButton("▶ START JAZDY (RUN)");
    btnStart->setMinimumHeight(34);
    btnStart->setStyleSheet("background-color: #1e6f3d; color: white; font-weight: bold;");

    auto *btnStop = new QPushButton("⏹ STOP ALL (IDLE) [Spacja]");
    btnStop->setMinimumHeight(34);
    btnStop->setStyleSheet("background-color: #9e2a2b; color: white; font-weight: bold;");

    auto *btnCalibrate = new QPushButton("⚙ Kalibruj czujniki linii");
    btnCalibrate->setMinimumHeight(28);

    auto *btnSpin = new QPushButton("🔍 Identyfikuj (SPIN 2s)");
    btnSpin->setMinimumHeight(28);

    connect(btnStart, &QPushButton::clicked, this, &MainWindow::sendStartCommand);
    connect(btnStop, &QPushButton::clicked, this, &MainWindow::sendStopCommand);
    connect(btnCalibrate, &QPushButton::clicked, this, &MainWindow::sendCalibrateCommand);
    connect(btnSpin, &QPushButton::clicked, this, &MainWindow::sendSpinCommand);

    actionGrid->addWidget(btnStart, 0, 0);
    actionGrid->addWidget(btnStop, 0, 1);
    actionGrid->addWidget(btnCalibrate, 1, 0);
    actionGrid->addWidget(btnSpin, 1, 1);
    rightLayout->addWidget(actionGroup);

    // 2. Parametry modelu IDM
    auto *paramGroup = new QGroupBox("Parametry modelu IDM (Zmiana na żywo)");
    auto *paramGrid = new QGridLayout(paramGroup);
    paramGrid->setContentsMargins(8, 6, 8, 6);

    chkFreespeed = new QCheckBox("Freespeed (v0):");
    chkFreespeed->setChecked(true);
    spinFreespeed = new QDoubleSpinBox();
    spinFreespeed->setRange(0.0, 100.0);
    spinFreespeed->setSingleStep(1.0);
    spinFreespeed->setValue(35.0);

    chkAcc = new QCheckBox("Przyspieszenie (acc):");
    chkAcc->setChecked(true);
    spinAcc = new QDoubleSpinBox();
    spinAcc->setRange(0.05, 10.0);
    spinAcc->setSingleStep(0.1);
    spinAcc->setDecimals(2);
    spinAcc->setValue(0.50);

    chkBrake = new QCheckBox("Hamowanie (brake):");
    chkBrake->setChecked(true);
    spinBrake = new QDoubleSpinBox();
    spinBrake->setRange(0.05, 10.0);
    spinBrake->setSingleStep(0.1);
    spinBrake->setDecimals(2);
    spinBrake->setValue(0.80);

    auto *btnSendParams = new QPushButton("📤 Wyślij parametry IDM");
    btnSendParams->setMinimumHeight(28);
    btnSendParams->setStyleSheet("background-color: #005fb8; color: white; font-weight: bold;");
    connect(btnSendParams, &QPushButton::clicked, this, &MainWindow::sendParametersCommand);

    paramGrid->addWidget(chkFreespeed, 0, 0);
    paramGrid->addWidget(spinFreespeed, 0, 1);
    paramGrid->addWidget(chkAcc, 1, 0);
    paramGrid->addWidget(spinAcc, 1, 1);
    paramGrid->addWidget(chkBrake, 2, 0);
    paramGrid->addWidget(spinBrake, 2, 1);
    paramGrid->addWidget(btnSendParams, 3, 0, 1, 2);
    rightLayout->addWidget(paramGroup);

    // 3. NOWOŚĆ: Parametry regulatora PID (Trzymanie linii)
    auto *pidGroup = new QGroupBox("Regulator PID – trzymanie linii (Tłumienie oscylacji)");
    auto *pidGrid = new QGridLayout(pidGroup);
    pidGrid->setContentsMargins(8, 6, 8, 6);

    chkKp = new QCheckBox("Proporcjonalny (Kp):");
    chkKp->setChecked(true);
    spinKp = new QDoubleSpinBox();
    spinKp->setRange(0.0, 1.0);
    spinKp->setDecimals(4);
    spinKp->setSingleStep(0.001);
    spinKp->setValue(0.0140);

    chkKi = new QCheckBox("Całkujący (Ki):");
    chkKi->setChecked(true);
    spinKi = new QDoubleSpinBox();
    spinKi->setRange(0.0, 1.0);
    spinKi->setDecimals(5);
    spinKi->setSingleStep(0.0001);
    spinKi->setValue(0.00000);

    chkKd = new QCheckBox("Różniczkujący (Kd):");
    chkKd->setChecked(true);
    spinKd = new QDoubleSpinBox();
    spinKd->setRange(0.0, 1.0);
    spinKd->setDecimals(5);
    spinKd->setSingleStep(0.0001);
    spinKd->setValue(0.00030);

    auto *btnSendPid = new QPushButton("🎛 Wyślij nastawy PID (kp, ki, kd)");
    btnSendPid->setMinimumHeight(28);
    btnSendPid->setStyleSheet("background-color: #2b7a78; color: white; font-weight: bold;");
    connect(btnSendPid, &QPushButton::clicked, this, &MainWindow::sendPidCommand);

    pidGrid->addWidget(chkKp, 0, 0);
    pidGrid->addWidget(spinKp, 0, 1);
    pidGrid->addWidget(chkKi, 1, 0);
    pidGrid->addWidget(spinKi, 1, 1);
    pidGrid->addWidget(chkKd, 2, 0);
    pidGrid->addWidget(spinKd, 2, 1);
    pidGrid->addWidget(btnSendPid, 3, 0, 1, 2);
    rightLayout->addWidget(pidGroup);

    // 4. Generator korka widmowego
    auto *jamGroup = new QGroupBox("Generator korka widmowego (Impuls hamowania)");
    auto *jamGrid = new QGridLayout(jamGroup);
    jamGrid->setContentsMargins(8, 6, 8, 6);

    jamGrid->addWidget(new QLabel("Zwolnij do prędkości:"), 0, 0);
    spinJamSpeed = new QDoubleSpinBox();
    spinJamSpeed->setRange(0.0, 60.0);
    spinJamSpeed->setValue(12.0);
    jamGrid->addWidget(spinJamSpeed, 0, 1);

    jamGrid->addWidget(new QLabel("Czas trwania [s]:"), 1, 0);
    spinJamDuration = new QDoubleSpinBox();
    spinJamDuration->setRange(0.5, 30.0);
    spinJamDuration->setSingleStep(0.5);
    spinJamDuration->setValue(3.0);
    jamGrid->addWidget(spinJamDuration, 1, 1);

    auto *btnJam = new QPushButton("⚠ Wywołaj zaburzenie na zaznaczonych");
    btnJam->setMinimumHeight(28);
    btnJam->setStyleSheet("background-color: #b5651d; color: white; font-weight: bold;");
    connect(btnJam, &QPushButton::clicked, this, &MainWindow::triggerPhantomJam);
    jamGrid->addWidget(btnJam, 2, 0, 1, 2);
    rightLayout->addWidget(jamGroup);

    // 5. Menedżer OTA
    auto *otaGroup = new QGroupBox("Bezprzewodowa aktualizacja kodu (OTA Update)");
    auto *otaLayout = new QVBoxLayout(otaGroup);
    otaLayout->setContentsMargins(8, 6, 8, 6);

    auto *fileRow = new QHBoxLayout();
    editOtaFilePath = new QLineEdit("apps/app_vision.py");
    auto *btnBrowse = new QPushButton("Przeglądaj...");
    connect(btnBrowse, &QPushButton::clicked, this, &MainWindow::browseOtaFile);
    fileRow->addWidget(editOtaFilePath);
    fileRow->addWidget(btnBrowse);
    otaLayout->addLayout(fileRow);

    auto *verRow = new QHBoxLayout();
    chkAutoVersionTag = new QCheckBox("Nadpisz APP_VERSION:");
    chkAutoVersionTag->setChecked(true);
    editCustomVersionTag = new QLineEdit("vision_1.0");
    verRow->addWidget(chkAutoVersionTag);
    verRow->addWidget(editCustomVersionTag);
    otaLayout->addLayout(verRow);

    auto *btnOtaSend = new QPushButton("🚀 Wgraj plik przez Wi-Fi (OTA_UPDATE)");
    btnOtaSend->setMinimumHeight(30);
    btnOtaSend->setStyleSheet("background-color: #5c2d91; color: white; font-weight: bold;");
    connect(btnOtaSend, &QPushButton::clicked, this, &MainWindow::sendOtaCommand);
    otaLayout->addWidget(btnOtaSend);

    lblOtaStats = new QLabel("Pobrania HTTP OTA w tej sesji: 0");
    otaLayout->addWidget(lblOtaStats);
    rightLayout->addWidget(otaGroup);

    rightLayout->addStretch();

    mainLayout->addLayout(leftLayout, 64);
    mainLayout->addLayout(rightLayout, 36);
    return tab;
}

QWidget* MainWindow::createVisionTab() {
    auto *tab = new QWidget(this);
    auto *vbox = new QVBoxLayout(tab);
    vbox->setContentsMargins(6, 6, 6, 6);
    vbox->setSpacing(6);

    // Wiersz 1: Kamera, korekta kąta i STOP
    auto *row1 = new QHBoxLayout();
    comboCamera = new QComboBox();
    comboResolution = new QComboBox();
    comboResolution->addItem("1280 x 720 (HD)", QSize(1280, 720));
    comboResolution->addItem("1920 x 1080 (FullHD)", QSize(1920, 1080));

    auto *btnRefreshCam = new QPushButton("🔄 Odśwież");
    btnToggleCamera = new QPushButton("▶ Start kamery");
    btnToggleCamera->setStyleSheet("background-color: #1e6f3d; color: white; font-weight: bold; padding: 4px 12px;");

    spinHeadingOffset = new QSpinBox();
    spinHeadingOffset->setRange(-180, 180);
    spinHeadingOffset->setValue(0);
    spinHeadingOffset->setSuffix("°");

    comboTurnSign = new QComboBox();
    comboTurnSign->addItem("Skręt normalny (+1)", 1);
    comboTurnSign->addItem("Skręt odwrócony (-1)", -1);

    auto *btnStopVis = new QPushButton("⏹ STOP WSZYSTKICH [Spacja]");
    btnStopVis->setStyleSheet("background-color: #9e2a2b; color: white; font-weight: bold; padding: 4px 12px;");

    connect(btnRefreshCam, &QPushButton::clicked, this, &MainWindow::refreshCameras);
    connect(btnToggleCamera, &QPushButton::clicked, this, &MainWindow::toggleCamera);
    connect(btnStopVis, &QPushButton::clicked, this, &MainWindow::stopVisionAll);

    row1->addWidget(new QLabel("Kamera:"));
    row1->addWidget(comboCamera);
    row1->addWidget(comboResolution);
    row1->addWidget(btnRefreshCam);
    row1->addWidget(btnToggleCamera);
    row1->addSpacing(12);
    row1->addWidget(new QLabel("Korekta ArUco:"));
    row1->addWidget(spinHeadingOffset);
    row1->addWidget(comboTurnSign);
    row1->addStretch();
    row1->addWidget(btnStopVis);
    vbox->addLayout(row1);

    // Wiersz 2: Sterowanie torem (Ósemka, Interaktywne koło, Kalibracja areny)
    auto *row2 = new QHBoxLayout();
    auto *btnFig8 = new QPushButton("∞ Ósemka (Wybrany)");
    auto *btnCircleSetup = new QPushButton("🎯 Dopasuj okrąg (Przeciągnij/Scroll)");
    btnCircleSetup->setStyleSheet("background-color: #b58900; color: white; font-weight: bold;");

    spinCircleRadius = new QSpinBox();
    spinCircleRadius->setRange(40, 600);
    spinCircleRadius->setSingleStep(10);
    spinCircleRadius->setValue(180);
    spinCircleRadius->setSuffix(" px");
    connect(spinCircleRadius, QOverload<int>::of(&QSpinBox::valueChanged), this, [this](int val) {
        circleRadius = static_cast<double>(val);
    });

    comboCircleDir = new QComboBox();
    comboCircleDir->addItem("↻ Zgodnie z zegarem", 1);
    comboCircleDir->addItem("↺ Przeciwnie do zegara", -1);

    auto *btnCircleRun = new QPushButton("⭕ Start po okręgu");
    btnCircleRun->setStyleSheet("background-color: #005fb8; color: white; font-weight: bold;");

    btnArenaCalib = new QPushButton("📐 Wyznacz arenę (4 rogi)");

    connect(btnFig8, &QPushButton::clicked, this, &MainWindow::startVisionFigureEight);
    connect(btnCircleSetup, &QPushButton::clicked, this, &MainWindow::startCircleSetup);
    connect(btnCircleRun, &QPushButton::clicked, this, &MainWindow::startVisionCircleAll);
    connect(btnArenaCalib, &QPushButton::clicked, this, &MainWindow::startArenaCalibration);

    row2->addWidget(btnFig8);
    row2->addSpacing(10);
    row2->addWidget(btnCircleSetup);
    row2->addWidget(new QLabel("Promień:"));
    row2->addWidget(spinCircleRadius);
    row2->addWidget(comboCircleDir);
    row2->addWidget(btnCircleRun);
    row2->addSpacing(15);
    row2->addWidget(btnArenaCalib);
    row2->addStretch();
    vbox->addLayout(row2);

    lblVisionStatus = new QLabel(
        "Mapowanie: PICO_X = ArUco #X. Wybierz robota w tabeli i kliknij na obraz (cel), "
        "użyj 'Dopasuj okrąg' do ustawienia na torze, albo uruchom symulację Ławicy ryb."
    );
    lblVisionStatus->setStyleSheet("color: #dcdcaa; font-weight: bold;");
    vbox->addWidget(lblVisionStatus);

    // Część główna: Podgląd kamery po lewej + Tabela i panel Ławicy ryb po prawej
    auto *contentRow = new QHBoxLayout();

    videoLabel = new ClickableVideoLabel("Kamera wyłączona – kliknij 'Start kamery'");
    videoLabel->setAlignment(Qt::AlignCenter);
    videoLabel->setMinimumSize(700, 460);
    videoLabel->setStyleSheet("background-color: #111111; color: #888888; font-size: 15px; border: 1px solid #333;");
    videoLabel->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    connect(videoLabel, &ClickableVideoLabel::imageClicked, this, &MainWindow::onVideoImageClicked);
    connect(videoLabel, &ClickableVideoLabel::imageDragged, this, &MainWindow::onVideoImageDragged);
    connect(videoLabel, &ClickableVideoLabel::wheelScrolled, this, &MainWindow::onVideoWheelScrolled);
    contentRow->addWidget(videoLabel, 1);

    // Prawa kolumna Zakładki 2
    auto *rightCol = new QVBoxLayout();
    rightCol->setSpacing(6);

    visionTable = new QTableWidget(0, 6, this);
    visionTable->setHorizontalHeaderLabels({"ArUco", "Nazwa", "IP", "Sieć", "Pozycja (X,Y,°)", "Kamera"});
    visionTable->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    visionTable->setSelectionBehavior(QAbstractItemView::SelectRows);
    visionTable->setSelectionMode(QAbstractItemView::SingleSelection);
    visionTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
    visionTable->setFixedWidth(450);
    connect(visionTable, &QTableWidget::itemSelectionChanged, this, &MainWindow::onVisionTableSelectionChanged);
    rightCol->addWidget(visionTable, 1);

    // Panel symulacji Ławicy ryb (flocking_simple.py w C++)
    auto *flockGroup = new QGroupBox("🐟 Symulacja ławicy ryb (Boids – parametry na żywo)");
    flockGroup->setFixedWidth(450);
    auto *fg = new QGridLayout(flockGroup);
    fg->setContentsMargins(8, 6, 8, 6);

    spinArenaW = new QDoubleSpinBox();
    spinArenaW->setRange(30.0, 2000.0);
    spinArenaW->setValue(flockingSim.getArenaWidthCm());
    spinArenaW->setSuffix(" cm");

    spinArenaH = new QDoubleSpinBox();
    spinArenaH->setRange(30.0, 2000.0);
    spinArenaH->setValue(flockingSim.getArenaHeightCm());
    spinArenaH->setSuffix(" cm");

    spinFlockSpeed = new QDoubleSpinBox();
    spinFlockSpeed->setRange(5.0, 80.0);
    spinFlockSpeed->setSingleStep(1.0);
    spinFlockSpeed->setValue(flockingSim.params.speedCmS);
    spinFlockSpeed->setSuffix(" cm/s");

    spinFlockTurnR = new QDoubleSpinBox();
    spinFlockTurnR->setRange(8.0, 100.0);
    spinFlockTurnR->setSingleStep(2.0);
    spinFlockTurnR->setValue(flockingSim.params.minTurnRadiusCm);
    spinFlockTurnR->setSuffix(" cm");

    spinFlockSep = new QDoubleSpinBox();
    spinFlockSep->setRange(0.0, 30.0);
    spinFlockSep->setSingleStep(0.5);
    spinFlockSep->setValue(flockingSim.params.wSep);

    spinFlockAlign = new QDoubleSpinBox();
    spinFlockAlign->setRange(0.0, 30.0);
    spinFlockAlign->setSingleStep(0.5);
    spinFlockAlign->setValue(flockingSim.params.wAlign);

    spinFlockCoh = new QDoubleSpinBox();
    spinFlockCoh->setRange(0.0, 30.0);
    spinFlockCoh->setSingleStep(0.2);
    spinFlockCoh->setValue(flockingSim.params.wCoh);

    spinPreviewCount = new QSpinBox();
    spinPreviewCount->setRange(1, 25);
    spinPreviewCount->setValue(8);

    auto *btnFlockPreview = new QPushButton("👁 Podgląd ławicy (Bez robotów)");
    btnFlockPreview->setMinimumHeight(30);
    btnFlockPreview->setStyleSheet("background-color: #2b7a78; color: white; font-weight: bold;");

    auto *btnFlockStart = new QPushButton("🐟 START ŁAWICY (Na robotach)");
    btnFlockStart->setMinimumHeight(32);
    btnFlockStart->setStyleSheet("background-color: #1e6f3d; color: white; font-weight: bold;");

    connect(spinArenaW, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &MainWindow::updateFlockingParamsFromUi);
    connect(spinArenaH, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &MainWindow::updateFlockingParamsFromUi);
    connect(spinFlockSpeed, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &MainWindow::updateFlockingParamsFromUi);
    connect(spinFlockTurnR, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &MainWindow::updateFlockingParamsFromUi);
    connect(spinFlockSep, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &MainWindow::updateFlockingParamsFromUi);
    connect(spinFlockAlign, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &MainWindow::updateFlockingParamsFromUi);
    connect(spinFlockCoh, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &MainWindow::updateFlockingParamsFromUi);
    connect(btnFlockPreview, &QPushButton::clicked, this, &MainWindow::startFlockingPreview);
    connect(btnFlockStart, &QPushButton::clicked, this, &MainWindow::startFlockingRobots);

    fg->addWidget(new QLabel("Arena Szer.:"), 0, 0);
    fg->addWidget(spinArenaW, 0, 1);
    fg->addWidget(new QLabel("Arena Wys.:"), 0, 2);
    fg->addWidget(spinArenaH, 0, 3);

    fg->addWidget(new QLabel("Prędkość:"), 1, 0);
    fg->addWidget(spinFlockSpeed, 1, 1);
    fg->addWidget(new QLabel("Min. R skrętu:"), 1, 2);
    fg->addWidget(spinFlockTurnR, 1, 3);

    fg->addWidget(new QLabel("Separacja:"), 2, 0);
    fg->addWidget(spinFlockSep, 2, 1);
    fg->addWidget(new QLabel("Wyrównanie:"), 2, 2);
    fg->addWidget(spinFlockAlign, 2, 3);

    fg->addWidget(new QLabel("Spójność:"), 3, 0);
    fg->addWidget(spinFlockCoh, 3, 1);
    fg->addWidget(new QLabel("Wirt. ryb:"), 3, 2);
    fg->addWidget(spinPreviewCount, 3, 3);

    fg->addWidget(btnFlockPreview, 4, 0, 1, 2);
    fg->addWidget(btnFlockStart, 4, 2, 1, 2);

    rightCol->addWidget(flockGroup);
    contentRow->addLayout(rightCol);

    vbox->addLayout(contentRow, 1);
    return tab;
}

// ==================== MAPOWANIE PICO_X <-> ARUCO ID X ====================

int MainWindow::extractNumberFromName(const QString &name) {
    static QRegularExpression re("(\\d+)");
    QRegularExpressionMatch match = re.match(name);
    if (match.hasMatch()) return match.captured(1).toInt();
    return -1;
}

QString MainWindow::findRobotUidByMarkerId(int markerId) const {
    for (auto it = uidToName.begin(); it != uidToName.end(); ++it) {
        if (extractNumberFromName(it.value()) == markerId) {
            return it.key();
        }
    }
    return QString();
}

void MainWindow::rebuildVisionTable() {
    // Pokazujemy wyłącznie aktywne roboty z floty (bez fałszywych duchów z szumu kamery)
    QMap<int, QString> markerToUid;
    for (auto it = fleet.begin(); it != fleet.end(); ++it) {
        int mId = extractNumberFromName(it.value().name);
        if (mId >= 0) {
            markerToUid[mId] = it.key();
        }
    }

    QList<int> ids = markerToUid.keys();
    std::sort(ids.begin(), ids.end());
    qint64 now = QDateTime::currentMSecsSinceEpoch();

    // Przebudowujemy wiersze tylko gdy zmieniła się lista robotów (nie zrywa zaznaczenia przy 30 FPS)
    bool structureChanged = (ids != lastVisionTableIds);
    if (structureChanged) {
        lastVisionTableIds = ids;
        visionTable->blockSignals(true);
        visionTable->setRowCount(ids.size());
        for (int r = 0; r < ids.size(); ++r) {
            for (int c = 0; c < 6; ++c) {
                auto *item = new QTableWidgetItem();
                item->setTextAlignment(Qt::AlignCenter);
                visionTable->setItem(r, c, item);
            }
        }
        visionTable->blockSignals(false);
    }

    for (int r = 0; r < ids.size(); ++r) {
        int mId = ids[r];
        QString uid = markerToUid[mId];
        const RobotInfo &ri = fleet[uid];
        QString netStatus = ((now - ri.lastSeenMs) < 5000) ? "🟢 ON" : "🔴 OFF";

        QString posStr = "--";
        QString camStatus = "Brak";
        if (visionTracks.contains(mId)) {
            const VisionTrack &vt = visionTracks[mId];
            if ((now - vt.lastSeenMs) < 600) {
                posStr = QString("%1,%2 (%3°)")
                             .arg(vt.centerX, 0, 'f', 0)
                             .arg(vt.centerY, 0, 'f', 0)
                             .arg(vt.angleDeg, 0, 'f', 0);
                camStatus = "👁 OK";
            }
        }

        QStringList cols = {QString::number(mId), ri.name, ri.ip, netStatus, posStr, camStatus};
        for (int c = 0; c < cols.size(); ++c) {
            if (auto *it = visionTable->item(r, c)) {
                it->setText(cols[c]);
            }
        }
    }
}

// ==================== OBSŁUGA KAMERY ARUCO, KOŁA I ŁAWICY ====================

void MainWindow::refreshCameras() {
    comboCamera->clear();
    for (int i = 0; i < 5; ++i) {
        cv::VideoCapture testCap(i, cv::CAP_DSHOW);
        if (testCap.isOpened()) {
            comboCamera->addItem(QString("Kamera %1").arg(i), i);
            testCap.release();
        }
    }
    if (comboCamera->count() == 0) {
        comboCamera->addItem("Kamera 0 (Domyślna)", 0);
    }
}

void MainWindow::toggleCamera() {
    if (cameraTimer->isActive()) {
        stopVisionAll();
        cameraTimer->stop();
        cameraCap.release();
        btnToggleCamera->setText("▶ Start kamery");
        btnToggleCamera->setStyleSheet("background-color: #1e6f3d; color: white; font-weight: bold; padding: 4px 12px;");
        videoLabel->setPixmap(QPixmap());
        videoLabel->setText("Kamera zatrzymana");
        return;
    }

    int camIdx = comboCamera->currentData().toInt();
    QSize res = comboResolution->currentData().toSize();

    if (!cameraCap.open(camIdx, cv::CAP_DSHOW)) {
        cameraCap.open(camIdx, cv::CAP_ANY);
    }
    if (!cameraCap.isOpened()) {
        logMessage("[KAMERA BŁĄD] Nie można otworzyć wybranej kamery!");
        return;
    }

    cameraCap.set(cv::CAP_PROP_FRAME_WIDTH, res.width());
    cameraCap.set(cv::CAP_PROP_FRAME_HEIGHT, res.height());
    cameraCap.set(cv::CAP_PROP_FPS, 30);

    cameraTimer->start(33);
    btnToggleCamera->setText("⏹ Stop kamery");
    btnToggleCamera->setStyleSheet("background-color: #9e2a2b; color: white; font-weight: bold; padding: 4px 12px;");
    logMessage(QString("[KAMERA] Uruchomiono kamerę %1 (%2x%3).").arg(camIdx).arg(res.width()).arg(res.height()));
}

double MainWindow::normalizeAngleDeg(double a) {
    double r = std::fmod(a + 180.0, 360.0);
    if (r < 0) r += 360.0;
    return r - 180.0;
}

void MainWindow::processCameraFrame() {
    cv::Mat frame;
    if (!cameraCap.read(frame) || frame.empty()) return;

    lastFrameW = frame.cols;
    lastFrameH = frame.rows;
    if (!circleCenterInitialized) {
        circleCenter = QPointF(frame.cols * 0.5, frame.rows * 0.5);
        circleCenterInitialized = true;
    }

    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

    // 1. Detekcja znaczników ArUco (DICT_4X4_50)
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    arucoDetector.detectMarkers(gray, corners, ids);

    qint64 now = QDateTime::currentMSecsSinceEpoch();
    double offsetDeg = spinHeadingOffset->value();

    for (size_t i = 0; i < ids.size(); ++i) {
        int mId = ids[i];
        if (mId < 0 || mId >= 25) continue;

        const auto &pts = corners[i];
        cv::Point2f center = 0.25f * (pts[0] + pts[1] + pts[2] + pts[3]);
        cv::Point2f topMid = 0.5f * (pts[0] + pts[1]);
        cv::Point2f vec = topMid - center;
        double rawAngle = std::atan2(vec.y, vec.x) * 180.0 / kPi;
        double heading = normalizeAngleDeg(rawAngle + offsetDeg);

        VisionTrack &tr = visionTracks[mId];
        tr.markerId = mId;
        tr.centerX = center.x;
        tr.centerY = center.y;
        tr.angleDeg = heading;
        tr.lastSeenMs = now;

        QString uid = findRobotUidByMarkerId(mId);
        bool hasOnlineRobot = (!uid.isEmpty() && fleet.contains(uid) && (now - fleet[uid].lastSeenMs) < 5000);
        cv::Scalar color = hasOnlineRobot ? cv::Scalar(0, 230, 0) : cv::Scalar(0, 165, 255);

        cv::Point cPt(std::lround(center.x), std::lround(center.y));
        double rad = heading * kPi / 180.0;
        cv::Point arrowEnd(std::lround(center.x + 42.0 * std::cos(rad)),
                           std::lround(center.y + 42.0 * std::sin(rad)));

        cv::circle(frame, cPt, 6, color, -1, cv::LINE_AA);
        cv::arrowedLine(frame, cPt, arrowEnd, color, 3, cv::LINE_AA, 0, 0.28);

        QString label = hasOnlineRobot
                            ? QString("PICO_%1 [.%2]").arg(mId).arg(fleet[uid].ip.split('.').last())
                            : QString("ArUco #%1").arg(mId);
        cv::putText(frame, label.toStdString(), cv::Point(cPt.x + 10, cPt.y - 10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.55, color, 2, cv::LINE_AA);
    }

    // 2. Krok sterowania wizyjnego / symulacji ławicy
    stepVisionControl(frame.cols, frame.rows);

    // 3. Rysowanie nakładek na obrazie
    // 3a) Ramka areny (podczas kalibracji i trybów ławicy)
    if (arenaCalibrating) {
        for (int i = 0; i < arenaClickPoints.size(); ++i) {
            cv::Point pt(std::lround(arenaClickPoints[i].x()), std::lround(arenaClickPoints[i].y()));
            cv::circle(frame, pt, 8, cv::Scalar(0, 220, 255), -1, cv::LINE_AA);
            cv::putText(frame, std::to_string(i + 1), cv::Point(pt.x + 10, pt.y - 8),
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 220, 255), 2, cv::LINE_AA);
            if (i > 0) {
                cv::Point prev(std::lround(arenaClickPoints[i - 1].x()), std::lround(arenaClickPoints[i - 1].y()));
                cv::line(frame, prev, pt, cv::Scalar(0, 220, 255), 2, cv::LINE_AA);
            }
        }
    } else if (visionMode == VisionMode::FlockingPreview || visionMode == VisionMode::FlockingRobots) {
        auto cornersN = flockingSim.getCornersNorm();
        std::vector<cv::Point> poly;
        for (const auto &c : cornersN) {
            poly.emplace_back(std::lround(c.x * frame.cols), std::lround(c.y * frame.rows));
        }
        cv::polylines(frame, poly, true, cv::Scalar(255, 220, 0), 2, cv::LINE_AA);
    }

    // 3b) Tryby jazdy (Punkt / Ósemka / Koło / Ławica)
    if (visionMode == VisionMode::Point && selectedMarkerId >= 0) {
        cv::circle(frame, cv::Point(visionPointTarget.x(), visionPointTarget.y()), 10, cv::Scalar(0, 0, 255), -1, cv::LINE_AA);
    } else if (visionMode == VisionMode::FigureEight && !figure8Path.isEmpty()) {
        for (int i = 0; i < figure8Path.size(); ++i) {
            QPointF p1 = figure8Path[i];
            QPointF p2 = figure8Path[(i + 1) % figure8Path.size()];
            cv::line(frame, cv::Point(p1.x(), p1.y()), cv::Point(p2.x(), p2.y()), cv::Scalar(255, 0, 255), 2, cv::LINE_AA);
        }
        cv::circle(frame, cv::Point(visionPointTarget.x(), visionPointTarget.y()), 9, cv::Scalar(0, 0, 255), -1, cv::LINE_AA);
    } else if (visionMode == VisionMode::CircleSetup || visionMode == VisionMode::Circle) {
        cv::Scalar cCol = (visionMode == VisionMode::CircleSetup) ? cv::Scalar(0, 255, 255) : cv::Scalar(0, 200, 255);
        cv::Point cc(std::lround(circleCenter.x()), std::lround(circleCenter.y()));
        cv::circle(frame, cc, std::lround(circleRadius), cCol, 2, cv::LINE_AA);
        cv::drawMarker(frame, cc, cCol, cv::MARKER_CROSS, 20, 2);

        // Strzałka wskazująca kierunek obiegu koła (CW / CCW) na górze okręgu
        int dir = comboCircleDir->currentData().toInt();
        cv::Point topPt(cc.x, std::max(15, cc.y - static_cast<int>(circleRadius)));
        cv::Point dirTip(topPt.x + dir * 40, topPt.y);
        cv::arrowedLine(frame, topPt, dirTip, cCol, 3, cv::LINE_AA, 0, 0.35);

        if (visionMode == VisionMode::CircleSetup) {
            cv::putText(frame, "DOPASOWANIE: przeciagnij = przesun, scroll = promien",
                        cv::Point(15, 32), cv::FONT_HERSHEY_SIMPLEX, 0.7, cCol, 2, cv::LINE_AA);
        }
    } else if (visionMode == VisionMode::FlockingPreview) {
        for (const auto &b : flockingSim.getBoids()) {
            cv::Point2d nPt = flockingSim.ownToNorm(b.x, b.y);
            cv::Point2d nHead = flockingSim.ownToNorm(b.x + 12.0 * std::cos(b.theta),
                                                      b.y + 12.0 * std::sin(b.theta));
            cv::Point p0(std::lround(nPt.x * frame.cols), std::lround(nPt.y * frame.rows));
            cv::Point p1(std::lround(nHead.x * frame.cols), std::lround(nHead.y * frame.rows));
            cv::circle(frame, p0, 8, cv::Scalar(0, 255, 0), -1, cv::LINE_AA);
            cv::arrowedLine(frame, p0, p1, cv::Scalar(0, 255, 0), 2, cv::LINE_AA, 0, 0.35);
            cv::putText(frame, "B" + std::to_string(b.markerId), cv::Point(p0.x + 9, p0.y - 6),
                        cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
        }
        cv::putText(frame, "PODGLAD LAWICY (bez robotow)",
                    cv::Point(15, 32), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
    } else if (visionMode == VisionMode::FlockingRobots) {
        for (auto it = flockingRobotTargetsPx.begin(); it != flockingRobotTargetsPx.end(); ++it) {
            cv::Point tPt(std::lround(it.value().x()), std::lround(it.value().y()));
            cv::circle(frame, tPt, 8, cv::Scalar(255, 0, 255), -1, cv::LINE_AA);
            cv::putText(frame, "T" + std::to_string(it.key()), cv::Point(tPt.x + 8, tPt.y + 4),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 255), 2, cv::LINE_AA);
            if (visionTracks.contains(it.key())) {
                cv::Point rPt(std::lround(visionTracks[it.key()].centerX),
                              std::lround(visionTracks[it.key()].centerY));
                cv::line(frame, rPt, tPt, cv::Scalar(255, 0, 255), 1, cv::LINE_AA);
            }
        }
    }

    // 4. Wyświetlenie klatki w QLabel
    cv::Mat rgb;
    cv::cvtColor(frame, rgb, cv::COLOR_BGR2RGB);
    videoLabel->setSourceSize(rgb.cols, rgb.rows);
    QImage qimg(rgb.data, rgb.cols, rgb.rows, int(rgb.step), QImage::Format_RGB888);
    videoLabel->setPixmap(QPixmap::fromImage(qimg).scaled(
        videoLabel->size(), Qt::KeepAspectRatio, Qt::FastTransformation));

    rebuildVisionTable();
}

void MainWindow::sendVisionDriveCmd(int markerId, double speed, double turn) {
    QString uid = findRobotUidByMarkerId(markerId);
    if (uid.isEmpty() || !fleet.contains(uid)) return;

    QJsonObject pkt{
        {"cmd", "VISION_DRIVE"},
        {"type", "cmd"},
        {"id", markerId},
        {"speed", std::round(speed * 1000.0) / 1000.0},
        {"turn",  std::round(turn  * 1000.0) / 1000.0}
    };
    sendRawUdp(QJsonDocument(pkt).toJson(QJsonDocument::Compact), fleet[uid].ip);
}

bool MainWindow::driveMarkerToPoint(VisionTrack &tr, QPointF target, double speedCap) {
    constexpr double ARRIVAL_RADIUS_PX = 18.0;
    constexpr double SLOWDOWN_RADIUS_PX = 260.0;
    constexpr double MIN_MOVING_SPEED = 0.08;
    constexpr double DRIVE_MAX_TURN = 0.20;
    constexpr double FULL_TURN_DEG = 100.0;
    constexpr double DEADBAND_DEG = 4.0;
    constexpr double MAX_TURN_RATIO = 0.85;

    double dx = target.x() - tr.centerX;
    double dy = target.y() - tr.centerY;
    double dist = std::hypot(dx, dy);

    if (dist <= ARRIVAL_RADIUS_PX) {
        sendVisionDriveCmd(tr.markerId, 0.0, 0.0);
        tr.smoothedSpeed = 0.0;
        tr.smoothedTurn = 0.0;
        return true;
    }

    double desiredDeg = std::atan2(dy, dx) * 180.0 / kPi;
    double error = normalizeAngleDeg(desiredDeg - tr.angleDeg);
    double ae = std::abs(error);

    double distScale = (dist >= SLOWDOWN_RADIUS_PX)
                           ? 1.0
                           : (0.60 + 0.40 * ((dist - ARRIVAL_RADIUS_PX) / (SLOWDOWN_RADIUS_PX - ARRIVAL_RADIUS_PX)));
    double headShape = 0.5 * (1.0 + std::cos(ae * kPi / 180.0));
    double headScale = 0.65 + 0.35 * headShape;

    double speed = std::max(MIN_MOVING_SPEED, speedCap * distScale * headScale);

    int turnSign = comboTurnSign->currentData().toInt();
    double turn = 0.0;
    if (ae > DEADBAND_DEG) {
        turn = std::clamp(DRIVE_MAX_TURN * error / FULL_TURN_DEG, -DRIVE_MAX_TURN, DRIVE_MAX_TURN) * turnSign;
    }

    double lim = MAX_TURN_RATIO * speed;
    turn = std::clamp(turn, -lim, lim);

    tr.smoothedSpeed = 0.80 * tr.smoothedSpeed + 0.20 * speed;
    tr.smoothedTurn  = 0.70 * tr.smoothedTurn  + 0.30 * turn;

    sendVisionDriveCmd(tr.markerId, tr.smoothedSpeed, tr.smoothedTurn);
    return false;
}

void MainWindow::stepVisionControl(int frameW, int frameH) {
    if (visionMode == VisionMode::None || visionMode == VisionMode::CircleSetup) return;
    qint64 now = QDateTime::currentMSecsSinceEpoch();

    if (visionMode == VisionMode::Point) {
        if (!visionTracks.contains(selectedMarkerId)) return;
        VisionTrack &tr = visionTracks[selectedMarkerId];
        if ((now - tr.lastSeenMs) > 400) {
            sendVisionDriveCmd(selectedMarkerId, 0.0, 0.0);
            return;
        }
        if (driveMarkerToPoint(tr, visionPointTarget, 0.36)) {
            visionMode = VisionMode::None;
            lblVisionStatus->setText(QString("Robot PICO_%1 dotarł do wskazanego celu!").arg(selectedMarkerId));
        }
    } else if (visionMode == VisionMode::FigureEight) {
        if (!visionTracks.contains(selectedMarkerId)) return;
        VisionTrack &tr = visionTracks[selectedMarkerId];
        if ((now - tr.lastSeenMs) > 400) {
            sendVisionDriveCmd(selectedMarkerId, 0.0, 0.0);
            return;
        }
        if (figure8Path.isEmpty()) {
            double cx = frameW / 2.0, cy = frameH / 2.0;
            double ax = 0.28 * frameW, ay = 0.18 * frameH;
            for (int i = 0; i < 120; ++i) {
                double t = 2.0 * kPi * i / 120.0;
                figure8Path.append(QPointF(cx + ax * std::sin(t), cy + ay * std::sin(2.0 * t)));
            }
            figure8Index = 0;
        }
        for (int k = 0; k < figure8Path.size(); ++k) {
            QPointF pt = figure8Path[figure8Index];
            if (std::hypot(pt.x() - tr.centerX, pt.y() - tr.centerY) > 105.0) break;
            figure8Index = (figure8Index + 1) % figure8Path.size();
        }
        visionPointTarget = figure8Path[figure8Index];
        driveMarkerToPoint(tr, visionPointTarget, 0.36);
    } else if (visionMode == VisionMode::Circle) {
        int dir = comboCircleDir->currentData().toInt();
        for (auto it = visionTracks.begin(); it != visionTracks.end(); ++it) {
            VisionTrack &tr = it.value();
            QString uid = findRobotUidByMarkerId(tr.markerId);
            if (uid.isEmpty() || !fleet.contains(uid)) continue;
            if ((now - tr.lastSeenMs) > 400) {
                sendVisionDriveCmd(tr.markerId, 0.0, 0.0);
                continue;
            }
            double phi = std::atan2(tr.centerY - circleCenter.y(), tr.centerX - circleCenter.x());
            double targetPhi = phi + dir * (135.0 / std::max(50.0, circleRadius));
            QPointF lookahead(circleCenter.x() + circleRadius * std::cos(targetPhi),
                              circleCenter.y() + circleRadius * std::sin(targetPhi));
            driveMarkerToPoint(tr, lookahead, 0.25);
        }
    } else if (visionMode == VisionMode::FlockingPreview) {
        double dt = (lastFlockingMs == 0) ? (1.0 / 30.0) : ((now - lastFlockingMs) / 1000.0);
        lastFlockingMs = now;
        flockingSim.step(dt);
    } else if (visionMode == VisionMode::FlockingRobots) {
        double dt = (lastFlockingMs == 0) ? (1.0 / 30.0) : ((now - lastFlockingMs) / 1000.0);
        lastFlockingMs = now;

        auto targetsNorm = flockingSim.step(dt);
        flockingRobotTargetsPx.clear();

        // Bezpieczeństwo: jeśli któryś robot z ławicy zniknie z kamery, wstrzymujemy go
        for (const auto &[mId, nPt] : targetsNorm) {
            QPointF tPx(nPt.x * frameW, nPt.y * frameH);
            flockingRobotTargetsPx[mId] = tPx;

            if (!visionTracks.contains(mId) || (now - visionTracks[mId].lastSeenMs) > 450) {
                sendVisionDriveCmd(mId, 0.0, 0.0);
                continue;
            }
            driveMarkerToPoint(visionTracks[mId], tPx, 0.40);
        }
    }
}

void MainWindow::onVisionTableSelectionChanged() {
    auto items = visionTable->selectedItems();
    if (!items.isEmpty()) {
        int row = items.first()->row();
        selectedMarkerId = visionTable->item(row, 0)->text().toInt();
        lblVisionStatus->setText(
            QString("Wybrano ArUco #%1 (PICO_%1). Kliknij na obraz kamery, aby wysłać robota w to miejsce.")
                .arg(selectedMarkerId));
    }
}

void MainWindow::onVideoImageClicked(double x, double y) {
    if (arenaCalibrating) {
        arenaClickPoints.append(QPointF(x, y));
        static const QStringList cornerNames = {"PRAWY GÓRNY", "PRAWY DOLNY", "LEWY DOLNY"};
        int n = arenaClickPoints.size();
        if (n < 4) {
            lblVisionStatus->setText(
                QString("Kalibracja areny (%1/4): kliknij %2 róg areny.").arg(n).arg(cornerNames[n - 1]));
        } else {
            std::array<cv::Point2f, 4> cornersN;
            for (int i = 0; i < 4; ++i) {
                cornersN[i] = cv::Point2f(static_cast<float>(arenaClickPoints[i].x() / lastFrameW),
                                          static_cast<float>(arenaClickPoints[i].y() / lastFrameH));
            }
            flockingSim.configureArena(cornersN, spinArenaW->value(), spinArenaH->value());
            saveArenaConfig();
            arenaCalibrating = false;
            arenaClickPoints.clear();
            btnArenaCalib->setText("📐 Wyznacz arenę (4 rogi)");
            lblVisionStatus->setText(
                QString("Zapisano kalibrację areny (%1 x %2 cm) w %3.")
                    .arg(spinArenaW->value(), 0, 'f', 0)
                    .arg(spinArenaH->value(), 0, 'f', 0)
                    .arg(ARENA_FILENAME));
        }
        return;
    }

    if (visionMode == VisionMode::CircleSetup || visionMode == VisionMode::Circle) {
        circleCenter = QPointF(x, y);
        circleCenterInitialized = true;
        return;
    }

    if (selectedMarkerId < 0) {
        lblVisionStatus->setText("Najpierw zaznacz robota w tabeli po prawej stronie!");
        return;
    }
    visionPointTarget = QPointF(x, y);
    visionMode = VisionMode::Point;
    lblVisionStatus->setText(
        QString("Robot PICO_%1 jedzie do punktu (%2, %3)...").arg(selectedMarkerId).arg(int(x)).arg(int(y)));
}

void MainWindow::onVideoImageDragged(double x, double y) {
    if (!arenaCalibrating && (visionMode == VisionMode::CircleSetup || visionMode == VisionMode::Circle)) {
        circleCenter = QPointF(x, y);
        circleCenterInitialized = true;
    }
}

void MainWindow::onVideoWheelScrolled(int delta) {
    if (visionMode == VisionMode::CircleSetup || visionMode == VisionMode::Circle) {
        int step = (delta > 0) ? 10 : -10;
        spinCircleRadius->setValue(spinCircleRadius->value() + step);
    }
}

void MainWindow::startVisionFigureEight() {
    if (selectedMarkerId < 0) {
        lblVisionStatus->setText("Najpierw zaznacz robota w tabeli po prawej stronie!");
        return;
    }
    arenaCalibrating = false;
    figure8Path.clear();
    visionMode = VisionMode::FigureEight;
    lblVisionStatus->setText(QString("Robot PICO_%1 rozpoczął jazdę po ósemce.").arg(selectedMarkerId));
}

void MainWindow::startCircleSetup() {
    stopVisionAll();
    arenaCalibrating = false;
    circleRadius = spinCircleRadius->value();
    if (!circleCenterInitialized) {
        circleCenter = QPointF(lastFrameW * 0.5, lastFrameH * 0.5);
        circleCenterInitialized = true;
    }
    visionMode = VisionMode::CircleSetup;
    lblVisionStatus->setText(
        "DOPASOWANIE OKRĘGU: przeciągnij myszką po obrazie, aby przesunąć środek; "
        "użyj scrolla myszy lub pola 'Promień', aby dopasować do toru, potem kliknij 'Start po okręgu'.");
}

void MainWindow::startVisionCircleAll() {
    arenaCalibrating = false;
    circleRadius = spinCircleRadius->value();
    if (!circleCenterInitialized) {
        circleCenter = QPointF(lastFrameW * 0.5, lastFrameH * 0.5);
        circleCenterInitialized = true;
    }
    visionMode = VisionMode::Circle;
    QString dirStr = (comboCircleDir->currentData().toInt() > 0) ? "zgodnie z zegarem" : "przeciwnie do zegara";
    lblVisionStatus->setText(
        QString("Wszystkie widoczne roboty jadą po okręgu (%1, R=%2 px). Możesz nadal korygować pozycję/promień.")
            .arg(dirStr).arg(int(circleRadius)));
}

void MainWindow::startArenaCalibration() {
    if (!cameraTimer->isActive()) {
        lblVisionStatus->setText("Najpierw kliknij 'Start kamery', aby zobaczyć arenę!");
        return;
    }
    if (arenaCalibrating) {
        arenaCalibrating = false;
        arenaClickPoints.clear();
        btnArenaCalib->setText("📐 Wyznacz arenę (4 rogi)");
        lblVisionStatus->setText("Anulowano wyznaczanie areny.");
        return;
    }
    stopVisionAll();
    arenaCalibrating = true;
    arenaClickPoints.clear();
    btnArenaCalib->setText("✖ Anuluj arenę");
    lblVisionStatus->setText("Kalibracja areny (0/4): kliknij LEWY GÓRNY róg (kolejność: LG -> PG -> PD -> LD).");
}

void MainWindow::updateFlockingParamsFromUi() {
    flockingSim.params.speedCmS = spinFlockSpeed->value();
    flockingSim.params.minTurnRadiusCm = spinFlockTurnR->value();
    flockingSim.params.wSep = spinFlockSep->value();
    flockingSim.params.wAlign = spinFlockAlign->value();
    flockingSim.params.wCoh = spinFlockCoh->value();
    flockingSim.configureArena(flockingSim.getCornersNorm(), spinArenaW->value(), spinArenaH->value());
}

void MainWindow::startFlockingPreview() {
    if (!cameraTimer->isActive()) {
        lblVisionStatus->setText("Najpierw kliknij 'Start kamery', aby wyświetlić podgląd ławicy na obrazie!");
        return;
    }
    stopVisionAll();
    arenaCalibrating = false;
    updateFlockingParamsFromUi();
    flockingSim.resetRandom(spinPreviewCount->value());
    lastFlockingMs = QDateTime::currentMSecsSinceEpoch();
    visionMode = VisionMode::FlockingPreview;
    lblVisionStatus->setText(
        QString("Podgląd ławicy ryb dla %1 wirtualnych agentów (parametry po prawej działają na żywo!).")
            .arg(spinPreviewCount->value()));
}

void MainWindow::startFlockingRobots() {
    if (!cameraTimer->isActive()) {
        lblVisionStatus->setText("Najpierw uruchom kamerę!");
        return;
    }
    stopVisionAll();
    arenaCalibrating = false;
    updateFlockingParamsFromUi();

    qint64 now = QDateTime::currentMSecsSinceEpoch();
    std::vector<std::tuple<int, double, double, double>> visibleRobots;
    for (auto it = visionTracks.begin(); it != visionTracks.end(); ++it) {
        const VisionTrack &vt = it.value();
        if ((now - vt.lastSeenMs) > 600) continue;
        QString uid = findRobotUidByMarkerId(vt.markerId);
        if (uid.isEmpty() || !fleet.contains(uid) || (now - fleet[uid].lastSeenMs) >= 5000) continue;

        visibleRobots.emplace_back(
            vt.markerId,
            vt.centerX / std::max(1, lastFrameW),
            vt.centerY / std::max(1, lastFrameH),
            vt.angleDeg);
    }

    if (visibleRobots.empty()) {
        lblVisionStatus->setText("Brak widocznych w kamerze robotów ONLINE (PICO_X z kodem ArUco #X)!");
        return;
    }

    flockingSim.initFromRobots(visibleRobots);
    lastFlockingMs = now;
    visionMode = VisionMode::FlockingRobots;
    lblVisionStatus->setText(
        QString("ŁAWICA RYB AKTYWNA dla %1 robotów! Zmieniaj wagi po prawej na żywo lub wciśnij Spację (STOP).")
            .arg(visibleRobots.size()));
}

void MainWindow::stopVisionAll() {
    visionMode = VisionMode::None;
    flockingRobotTargetsPx.clear();
    for (auto it = visionTracks.begin(); it != visionTracks.end(); ++it) {
        it.value().smoothedSpeed = 0.0;
        it.value().smoothedTurn = 0.0;
        sendVisionDriveCmd(it.key(), 0.0, 0.0);
    }
    if (lblVisionStatus) {
        lblVisionStatus->setText("Zatrzymano wszystkie roboty w trybie wizyjnym.");
    }
}

// ==================== WBUDOWANY SERWER DHCP (BEZ FILTRA MAC, Z IZOLACJĄ INTERFEJSU) ====================

void MainWindow::toggleDhcpServer(bool enabled) {
    if (enabled) {
        if (udpDhcpSocket->bind(QHostAddress::AnyIPv4, DHCP_SERVER_PORT, QUdpSocket::ShareAddress | QUdpSocket::ReuseAddressHint)) {
            logMessage("[DHCP] Serwer DHCP działa w tle na porcie UDP 67 (Pula: " + SUBNET_PREFIX + "21 - .250).");
            checkNetworkSetup();
        } else {
            logMessage("[DHCP BŁĄD] Port UDP 67 zajęty! Uruchom jako Administrator lub wyłącz inny serwer DHCP.");
            actDhcpEnable->setChecked(false);
        }
    } else {
        udpDhcpSocket->close();
        logMessage("[DHCP] Serwer DHCP został wyłączony z menu górnego.");
    }
}

static const QStringList kUbiquitiOui = {
    "6c:63:f8", "74:83:c2", "f0:9f:c2", "b4:fb:e4", "24:5a:4c", "80:2a:a8"
};

static bool macHasPrefix(const QString &mac, const QStringList &prefixes) {
    for (const QString &prefix : prefixes) {
        if (mac.startsWith(prefix, Qt::CaseInsensitive))
            return true;
    }
    return false;
}

bool MainWindow::findServerInterface(QNetworkAddressEntry *entry, int *ifaceIndex) const {
    for (const QNetworkInterface &iface : QNetworkInterface::allInterfaces()) {
        if (!(iface.flags() & QNetworkInterface::IsUp)) continue;
        for (const QNetworkAddressEntry &e : iface.addressEntries()) {
            if (e.ip().protocol() == QAbstractSocket::IPv4Protocol && e.ip().toString() == SERVER_ETH_IP) {
                if (entry) *entry = e;
                if (ifaceIndex) *ifaceIndex = iface.index();
                return true;
            }
        }
    }
    return false;
}

void MainWindow::checkNetworkSetup() {
    int owners = 0;
    QStringList sameSubnet;
    for (const QNetworkInterface &iface : QNetworkInterface::allInterfaces()) {
        if (!(iface.flags() & QNetworkInterface::IsUp)) continue;
        for (const QNetworkAddressEntry &e : iface.addressEntries()) {
            if (e.ip().protocol() != QAbstractSocket::IPv4Protocol) continue;
            const QString ip = e.ip().toString();
            if (ip == SERVER_ETH_IP)
                owners++;
            else if (ip.startsWith(SUBNET_PREFIX))
                sameSubnet << QString("%1 (%2)").arg(iface.humanReadableName(), ip);
        }
    }

    if (owners == 0) {
        logMessage(QString("[SIEĆ UWAGA] Żadna aktywna karta nie ma adresu %1 – serwer DHCP jest wstrzymany do czasu uruchomienia skryptu START.")
                       .arg(SERVER_ETH_IP));
    } else if (owners > 1) {
        logMessage(QString("[SIEĆ UWAGA] Adres %1 jest ustawiony na %2 kartach naraz – uruchom skrypt RESET dla nieużywanej karty.")
                       .arg(SERVER_ETH_IP).arg(owners));
    }
    if (!sameSubnet.isEmpty()) {
        logMessage(QString("[SIEĆ UWAGA] Inna karta jest w tej samej podsieci co roboty: %1.")
                       .arg(sameSubnet.join(", ")));
    }
}

QString MainWindow::allocateIpForMac(const QString &mac) {
    if (macToIp.contains(mac)) {
        return macToIp[mac];
    }

    const QString apIp = SUBNET_PREFIX + "10";
    if (macHasPrefix(mac, kUbiquitiOui) && !macToIp.values().contains(apIp)) {
        macToIp[mac] = apIp;
        saveRobotDatabase();
        refreshTableStatus();
        return apIp;
    }

    if (nextDhcpOctet < 21) {
        nextDhcpOctet = 21;
    }

    QString assignedIp;
    while (nextDhcpOctet < 250) {
        QString candidate = SUBNET_PREFIX + QString::number(nextDhcpOctet++);
        if (!macToIp.values().contains(candidate)) {
            assignedIp = candidate;
            break;
        }
    }

    if (assignedIp.isEmpty()) {
        logMessage(QString("[DHCP BŁĄD] Pula adresów wyczerpana – usuń stare wpisy z %1.").arg(DB_FILENAME));
        return QString();
    }

    macToIp[mac] = assignedIp;
    saveRobotDatabase();
    refreshTableStatus();
    return assignedIp;
}

void MainWindow::onDhcpReadyRead() {
    while (udpDhcpSocket->hasPendingDatagrams()) {
        QNetworkDatagram datagram = udpDhcpSocket->receiveDatagram();

        // Izolacja: odpowiadamy tylko wtedy, gdy karta z SERVER_ETH_IP jest aktywna
        // i pakiet przyszedł właśnie z tej karty (chroni sieć domową).
        QNetworkAddressEntry serverEntry;
        int serverIfaceIdx = -1;
        if (!findServerInterface(&serverEntry, &serverIfaceIdx))
            continue;
        if (datagram.interfaceIndex() != 0 && static_cast<int>(datagram.interfaceIndex()) != serverIfaceIdx)
            continue;

        QByteArray req = datagram.data();
        if (req.size() < 240 || static_cast<quint8>(req[0]) != 1)
            continue;

        if (static_cast<quint8>(req[236]) != 0x63 || static_cast<quint8>(req[237]) != 0x82 ||
            static_cast<quint8>(req[238]) != 0x53 || static_cast<quint8>(req[239]) != 0x63)
            continue;

        QByteArray xid = req.mid(4, 4);
        QByteArray chaddr = req.mid(28, 16);
        QString macStr = QString("%1:%2:%3:%4:%5:%6")
                             .arg(static_cast<quint8>(chaddr[0]), 2, 16, QChar('0'))
                             .arg(static_cast<quint8>(chaddr[1]), 2, 16, QChar('0'))
                             .arg(static_cast<quint8>(chaddr[2]), 2, 16, QChar('0'))
                             .arg(static_cast<quint8>(chaddr[3]), 2, 16, QChar('0'))
                             .arg(static_cast<quint8>(chaddr[4]), 2, 16, QChar('0'))
                             .arg(static_cast<quint8>(chaddr[5]), 2, 16, QChar('0')).toLower();

        quint8 dhcpMsgType = 0;
        bool hasServerId = false;
        quint32 requestedServerId = 0;
        int idx = 240;
        while (idx < req.size()) {
            quint8 opt = static_cast<quint8>(req[idx++]);
            if (opt == 255) break;
            if (opt == 0) continue;
            if (idx >= req.size()) break;
            quint8 len = static_cast<quint8>(req[idx++]);
            if (idx + len > req.size()) break;
            if (opt == 53 && len >= 1) {
                dhcpMsgType = static_cast<quint8>(req[idx]);
            } else if (opt == 54 && len == 4) {
                hasServerId = true;
                requestedServerId = qFromBigEndian<quint32>(reinterpret_cast<const uchar *>(req.constData() + idx));
            }
            idx += len;
        }

        if (dhcpMsgType != 1 && dhcpMsgType != 3) continue;
        if (dhcpMsgType == 3 && hasServerId && requestedServerId != QHostAddress(SERVER_ETH_IP).toIPv4Address())
            continue;

        QString clientIpStr = allocateIpForMac(macStr);
        if (clientIpStr.isEmpty()) continue;

        quint32 clientIpInt = qToBigEndian(QHostAddress(clientIpStr).toIPv4Address());
        quint32 serverIpInt = qToBigEndian(QHostAddress(SERVER_ETH_IP).toIPv4Address());
        quint32 maskInt     = qToBigEndian(QHostAddress("255.255.255.0").toIPv4Address());

        QByteArray resp(240, 0);
        resp[0] = 2; resp[1] = 1; resp[2] = 6; resp[3] = 0;
        resp.replace(4, 4, xid);
        resp[10] = static_cast<char>(0x80);

        memcpy(resp.data() + 16, &clientIpInt, 4);
        memcpy(resp.data() + 20, &serverIpInt, 4);
        resp.replace(28, 16, chaddr);

        resp[236] = static_cast<char>(0x63);
        resp[237] = static_cast<char>(0x82);
        resp[238] = static_cast<char>(0x53);
        resp[239] = static_cast<char>(0x63);

        quint8 replyType = (dhcpMsgType == 1) ? 2 : 5;
        resp.append(static_cast<char>(53)); resp.append(static_cast<char>(1)); resp.append(static_cast<char>(replyType));
        resp.append(static_cast<char>(54)); resp.append(static_cast<char>(4)); resp.append(reinterpret_cast<const char*>(&serverIpInt), 4);

        quint32 leaseTime = qToBigEndian(86400u);
        resp.append(static_cast<char>(51)); resp.append(static_cast<char>(4)); resp.append(reinterpret_cast<const char*>(&leaseTime), 4);
        resp.append(static_cast<char>(1));  resp.append(static_cast<char>(4)); resp.append(reinterpret_cast<const char*>(&maskInt), 4);
        resp.append(static_cast<char>(3));  resp.append(static_cast<char>(4)); resp.append(reinterpret_cast<const char*>(&serverIpInt), 4);
        resp.append(static_cast<char>(255));

        if (resp.size() < 300)
            resp.append(QByteArray(300 - resp.size(), 0));

        if (!serverEntry.broadcast().isNull())
            udpDhcpSocket->writeDatagram(resp, serverEntry.broadcast(), DHCP_CLIENT_PORT);
        udpDhcpSocket->writeDatagram(resp, QHostAddress::Broadcast, DHCP_CLIENT_PORT);

        if (replyType == 5) {
            logMessage(QString("[DHCP ACK] Przydzielono IP %1 dla urządzenia MAC: %2").arg(clientIpStr, macStr));
        }
    }
}

// ==================== OBSŁUGA TELEMETRII I OTA ====================

void MainWindow::onUdpReadyRead() {
    while (udpListenSocket->hasPendingDatagrams()) {
        QNetworkDatagram datagram = udpListenSocket->receiveDatagram();
        QString senderIp = QHostAddress(datagram.senderAddress().toIPv4Address()).toString();

        QJsonParseError err;
        QJsonDocument doc = QJsonDocument::fromJson(datagram.data(), &err);
        if (err.error != QJsonParseError::NoError || !doc.isObject()) continue;

        QJsonObject obj = doc.object();
        if (obj.value("type").toString() != "HELLO") continue;

        QString uid = obj.value("id").toString();
        if (uid.isEmpty()) continue;

        if (!uidToName.contains(uid)) {
            QString newName = QString("PICO_%1").arg(uidToName.size() + 1);
            uidToName[uid] = newName;
            saveRobotDatabase();
            logMessage(QString("[NOWY ROBOT] Zarejestrowano %1 (UID: %2, IP: %3)").arg(newName, uid, senderIp));
        }

        RobotInfo &r = fleet[uid];
        QString newVer = obj.value("ver").toString("?");
        if (!r.version.isEmpty() && r.version != newVer) {
            logMessage(QString("[SUKCES OTA] %1 zgłosił się z nową wersją kodu: v%2!").arg(uidToName[uid], newVer));
        }

        r.uid = uid;
        r.name = uidToName[uid];
        r.ip = senderIp;
        r.version = newVer;
        r.lastSeenMs = QDateTime::currentMSecsSinceEpoch();

        if (obj.contains("params") && obj["params"].isObject()) {
            QJsonObject p = obj["params"].toObject();
            r.mode = p.value("mode").toString(r.mode);
            r.freespeed = p.value("freespeed").toDouble(r.freespeed);
            r.acc = p.value("acc").toDouble(r.acc);
            r.brake = p.value("brake").toDouble(r.brake);
            r.dist = p.value("dist").toDouble(r.dist);
            r.spd = p.value("spd").toDouble(r.spd);
        }

        updateTableRow(uid);
    }
}

void MainWindow::onNewHttpConnection() {
    while (httpOtaServer->hasPendingConnections()) {
        QTcpSocket *client = httpOtaServer->nextPendingConnection();
        QString clientIp = QHostAddress(client->peerAddress().toIPv4Address()).toString();

        connect(client, &QTcpSocket::readyRead, this, [this, client, clientIp]() {
            QByteArray req = client->readAll();
            if (!req.startsWith("GET ")) {
                client->close();
                return;
            }
            QString filePath = editOtaFilePath->text().trimmed();
            QFile file(filePath);
            if (!file.open(QIODevice::ReadOnly)) {
                logMessage(QString("[BŁĄD OTA] Nie można otworzyć pliku: %1").arg(filePath));
                client->write("HTTP/1.0 404 Not Found\r\nConnection: close\r\n\r\n");
                client->flush();
                client->close();
                return;
            }
            QByteArray codeBytes = file.readAll();
            file.close();

            if (chkAutoVersionTag->isChecked() && !editCustomVersionTag->text().trimmed().isEmpty()) {
                QString codeStr = QString::fromUtf8(codeBytes);
                QString newTag = editCustomVersionTag->text().trimmed();
                QRegularExpression re(R"(APP_VERSION\s*=\s*["'][^"']*["'])");
                codeStr.replace(re, QString("APP_VERSION = \"%1\"").arg(newTag));
                codeBytes = codeStr.toUtf8();
            }

            QByteArray response = "HTTP/1.0 200 OK\r\n"
                                  "Content-Type: text/plain; charset=utf-8\r\n"
                                  "Connection: close\r\n\r\n" + codeBytes;

            client->write(response);
            client->flush();
            client->disconnectFromHost();

            otaDownloadsCount++;
            lblOtaStats->setText(QString("Pobrania HTTP OTA w tej sesji: %1").arg(otaDownloadsCount));
            logMessage(QString("[HTTP OTA] ---> Robot %1 pobrał plik '%2' (%3 B)")
                           .arg(clientIp, QFileInfo(filePath).fileName()).arg(codeBytes.size()));
        });

        connect(client, &QTcpSocket::disconnected, client, &QTcpSocket::deleteLater);
    }
}

void MainWindow::sendRawUdp(const QByteArray &data, const QString &ip) {
    udpCommandSocket->writeDatagram(data, QHostAddress(ip), UDP_COMMAND_PORT);
}

QList<QString> MainWindow::getTargetUids(bool forceAll) const {
    QList<QString> targets;
    for (auto it = fleet.begin(); it != fleet.end(); ++it) {
        if (forceAll || it.value().selected) {
            targets.append(it.key());
        }
    }
    return targets;
}

void MainWindow::sendJsonToTargets(const QJsonObject &payload, bool forceAll) {
    QJsonObject stamped = payload;
    stamped["seq"] = static_cast<qint64>(++commandSeq);
    QByteArray raw = QJsonDocument(stamped).toJson(QJsonDocument::Compact);

    if (forceAll) {
        sendRawUdp(raw, "255.255.255.255");
        for (const QNetworkInterface &iface : QNetworkInterface::allInterfaces()) {
            for (const QNetworkAddressEntry &entry : iface.addressEntries()) {
                if (!entry.broadcast().isNull())
                    sendRawUdp(raw, entry.broadcast().toString());
            }
        }
        for (const auto &r : fleet) {
            sendRawUdp(raw, r.ip);
        }
        logMessage(QString("--> Wysłano do WSZYSTKICH (%1): %2")
                       .arg(fleet.size()).arg(QString::fromUtf8(raw)));
    } else {
        QList<QString> targets = getTargetUids(false);
        if (targets.isEmpty()) {
            logMessage("UWAGA: Żaden robot nie jest zaznaczony w tabeli!");
            return;
        }
        QStringList names;
        for (const QString &uid : targets) {
            sendRawUdp(raw, fleet[uid].ip);
            names << fleet[uid].name;
        }
        logMessage(QString("--> Wysłano do [%1]: %2")
                       .arg(names.join(", "), QString::fromUtf8(raw)));
    }
}

// ==================== AKCJE PRZYCISKÓW ZAKŁADKI 1 ====================

void MainWindow::sendStartCommand() {
    sendJsonToTargets(QJsonObject{{"params", QJsonObject{{"mode", "RUN"}}}});
}

void MainWindow::sendStopCommand() {
    sendJsonToTargets(QJsonObject{{"params", QJsonObject{{"mode", "IDLE"}}}}, true);
}

void MainWindow::sendCalibrateCommand() {
    sendJsonToTargets(QJsonObject{{"cmd", "CALIBRATE"}});
}

void MainWindow::sendSpinCommand() {
    sendJsonToTargets(QJsonObject{{"cmd", "SPIN"}, {"duration", 2.0}, {"power", 50}});
}

void MainWindow::sendParametersCommand() {
    QJsonObject params;
    if (chkFreespeed->isChecked()) params["freespeed"] = spinFreespeed->value();
    if (chkAcc->isChecked())       params["acc"]       = spinAcc->value();
    if (chkBrake->isChecked())     params["brake"]     = spinBrake->value();

    if (params.isEmpty()) {
        logMessage("Zaznacz przynajmniej jeden parametr IDM (freespeed, acc lub brake)!");
        return;
    }
    sendJsonToTargets(QJsonObject{{"params", params}});
}

void MainWindow::sendPidCommand() {
    QJsonObject params;
    if (chkKp->isChecked()) params["kp"] = spinKp->value();
    if (chkKi->isChecked()) params["ki"] = spinKi->value();
    if (chkKd->isChecked()) params["kd"] = spinKd->value();

    if (params.isEmpty()) {
        logMessage("Zaznacz przynajmniej jeden parametr PID (Kp, Ki lub Kd)!");
        return;
    }
    sendJsonToTargets(QJsonObject{{"params", params}});
}

void MainWindow::triggerPhantomJam() {
    QList<QString> targets = getTargetUids(false);
    if (targets.isEmpty()) {
        logMessage("Zaznacz w tabeli robota, który ma wywołać korek!");
        return;
    }

    double lowSpeed = spinJamSpeed->value();
    int durationMs = static_cast<int>(spinJamDuration->value() * 1000.0);

    for (const QString &uid : targets) {
        double previousSpeed = fleet[uid].freespeed;
        QString ip = fleet[uid].ip;
        QString name = fleet[uid].name;

        QJsonObject slowPayload{{"params", QJsonObject{{"freespeed", lowSpeed}}}};
        sendRawUdp(QJsonDocument(slowPayload).toJson(QJsonDocument::Compact), ip);
        logMessage(QString("[ZABURZENIE] %1 zwalnia do freespeed=%2 na %3 s...")
                       .arg(name).arg(lowSpeed).arg(spinJamDuration->value()));

        QTimer::singleShot(durationMs, this, [this, uid, previousSpeed]() {
            if (!fleet.contains(uid)) return;
            QJsonObject restorePayload{{"params", QJsonObject{{"freespeed", previousSpeed}}}};
            sendRawUdp(QJsonDocument(restorePayload).toJson(QJsonDocument::Compact), fleet[uid].ip);
            logMessage(QString("[ZABURZENIE KONIEC] %1 wraca do freespeed=%2")
                           .arg(fleet[uid].name).arg(previousSpeed));
        });
    }
}

void MainWindow::browseOtaFile() {
    QString file = QFileDialog::getOpenFileName(this, "Wybierz skrypt Pythona dla robotów", "", "Python Files (*.py)");
    if (!file.isEmpty()) {
        editOtaFilePath->setText(file);
        editCustomVersionTag->setText(QFileInfo(file).baseName());
    }
}

void MainWindow::sendOtaCommand() {
    QString filePath = editOtaFilePath->text().trimmed();
    if (!QFileInfo::exists(filePath)) {
        logMessage(QString("BŁĄD: Plik '%1' nie istnieje! Wybierz poprawny plik .py.").arg(filePath));
        return;
    }
    sendJsonToTargets(QJsonObject{{"cmd", "OTA_UPDATE"}, {"port", HTTP_OTA_PORT}});
}

// ==================== ZARZĄDZANIE TABELĄ I BAZAMI JSON ====================

void MainWindow::updateTableRow(const QString &uid) {
    updatingTableProgrammatically = true;
    const RobotInfo &r = fleet[uid];

    int row = -1;
    for (int i = 0; i < fleetTable->rowCount(); ++i) {
        if (fleetTable->item(i, 0)->data(Qt::UserRole).toString() == uid) {
            row = i;
            break;
        }
    }

    if (row == -1) {
        row = fleetTable->rowCount();
        fleetTable->insertRow(row);

        auto *chkItem = new QTableWidgetItem();
        chkItem->setFlags(Qt::ItemIsUserCheckable | Qt::ItemIsEnabled | Qt::ItemIsSelectable);
        chkItem->setCheckState(r.selected ? Qt::Checked : Qt::Unchecked);
        chkItem->setData(Qt::UserRole, uid);
        fleetTable->setItem(row, 0, chkItem);

        for (int col = 1; col < 10; ++col) {
            auto *item = new QTableWidgetItem();
            if (col != 1) item->setFlags(item->flags() & ~Qt::ItemIsEditable);
            item->setTextAlignment(Qt::AlignCenter);
            fleetTable->setItem(row, col, item);
        }
    }

    fleetTable->item(row, 1)->setText(r.name);
    fleetTable->item(row, 2)->setText(r.ip);
    fleetTable->item(row, 3)->setText("v" + r.version);
    fleetTable->item(row, 5)->setText(r.mode);
    fleetTable->item(row, 5)->setForeground(r.mode.contains("RUN") ? QColor("#4ec9b0") : QColor("#ce9178"));
    fleetTable->item(row, 6)->setText(QString::number(r.dist, 'f', 1));
    fleetTable->item(row, 7)->setText(QString::number(r.spd, 'f', 1));
    fleetTable->item(row, 8)->setText(QString::number(r.freespeed, 'f', 1));
    fleetTable->item(row, 9)->setText(QString("%1 / %2").arg(r.acc, 0, 'f', 2).arg(r.brake, 0, 'f', 2));

    updatingTableProgrammatically = false;
    refreshTableStatus();
}

void MainWindow::refreshTableStatus() {
    updatingTableProgrammatically = true;
    qint64 now = QDateTime::currentMSecsSinceEpoch();
    int onlineCount = 0;
    int selectedCount = 0;

    for (int row = 0; row < fleetTable->rowCount(); ++row) {
        QString uid = fleetTable->item(row, 0)->data(Qt::UserRole).toString();
        if (!fleet.contains(uid)) continue;

        const RobotInfo &r = fleet[uid];
        if (r.selected) selectedCount++;

        qint64 ageSec = (now - r.lastSeenMs) / 1000;
        QTableWidgetItem *statusItem = fleetTable->item(row, 4);

        if (ageSec < 5) {
            onlineCount++;
            statusItem->setText(QString("🟢 ONLINE (%1s)").arg(ageSec));
            statusItem->setForeground(QColor("#6a9955"));
        } else {
            statusItem->setText(QString("🔴 OFFLINE (%1s)").arg(ageSec));
            statusItem->setForeground(QColor("#f44747"));
        }
    }

    fleetSummaryLabel->setText(QString("Widoczne roboty: %1 | 🟢 Online: %2 | ☑ Zaznaczone: %3 | 🌐 Dzierżawy DHCP: %4")
                                   .arg(fleet.size()).arg(onlineCount).arg(selectedCount).arg(macToIp.size()));
    updatingTableProgrammatically = false;

    if (!cameraTimer->isActive()) {
        rebuildVisionTable();
    }
}

void MainWindow::onTableItemChanged(QTableWidgetItem *item) {
    if (updatingTableProgrammatically) return;

    int row = item->row();
    QString uid = fleetTable->item(row, 0)->data(Qt::UserRole).toString();
    if (!fleet.contains(uid)) return;

    if (item->column() == 0) {
        fleet[uid].selected = (item->checkState() == Qt::Checked);
        refreshTableStatus();
    } else if (item->column() == 1) {
        QString newName = item->text().trimmed();
        if (!newName.isEmpty() && newName != fleet[uid].name) {
            fleet[uid].name = newName;
            uidToName[uid] = newName;
            saveRobotDatabase();
            rebuildVisionTable();
            logMessage(QString("Zmieniono nazwę robota (%1) na: %2 (ArUco #%3)")
                           .arg(uid, newName).arg(extractNumberFromName(newName)));
        }
    }
}

void MainWindow::selectAllRobots() {
    updatingTableProgrammatically = true;
    for (int row = 0; row < fleetTable->rowCount(); ++row) {
        fleetTable->item(row, 0)->setCheckState(Qt::Checked);
        fleet[fleetTable->item(row, 0)->data(Qt::UserRole).toString()].selected = true;
    }
    updatingTableProgrammatically = false;
    refreshTableStatus();
}

void MainWindow::deselectAllRobots() {
    updatingTableProgrammatically = true;
    for (int row = 0; row < fleetTable->rowCount(); ++row) {
        fleetTable->item(row, 0)->setCheckState(Qt::Unchecked);
        fleet[fleetTable->item(row, 0)->data(Qt::UserRole).toString()].selected = false;
    }
    updatingTableProgrammatically = false;
    refreshTableStatus();
}

void MainWindow::selectOnlineRobots() {
    updatingTableProgrammatically = true;
    qint64 now = QDateTime::currentMSecsSinceEpoch();
    for (int row = 0; row < fleetTable->rowCount(); ++row) {
        QString uid = fleetTable->item(row, 0)->data(Qt::UserRole).toString();
        bool isOnline = ((now - fleet[uid].lastSeenMs) / 1000) < 5;
        fleetTable->item(row, 0)->setCheckState(isOnline ? Qt::Checked : Qt::Unchecked);
        fleet[uid].selected = isOnline;
    }
    updatingTableProgrammatically = false;
    refreshTableStatus();
}

void MainWindow::loadRobotDatabase() {
    QFile file(DB_FILENAME);
    if (!file.open(QIODevice::ReadOnly)) return;
    QJsonDocument doc = QJsonDocument::fromJson(file.readAll());
    if (!doc.isObject()) return;

    QJsonObject obj = doc.object();
    for (auto it = obj.begin(); it != obj.end(); ++it) {
        if (it.key() == "dhcp_leases" && it.value().isObject()) {
            QJsonObject leases = it.value().toObject();
            for (auto lit = leases.begin(); lit != leases.end(); ++lit) {
                const QString ip = lit.value().toString();
                if (ip.startsWith(SUBNET_PREFIX))
                    macToIp[lit.key()] = ip;
            }
        } else if (it.value().isString()) {
            uidToName[it.key()] = it.value().toString();
        }
    }
}

void MainWindow::saveRobotDatabase() {
    QJsonObject obj;
    for (auto it = uidToName.begin(); it != uidToName.end(); ++it) {
        obj[it.key()] = it.value();
    }
    QJsonObject leases;
    for (auto it = macToIp.begin(); it != macToIp.end(); ++it) {
        leases[it.key()] = it.value();
    }
    obj["dhcp_leases"] = leases;

    QFile file(DB_FILENAME);
    if (file.open(QIODevice::WriteOnly)) {
        file.write(QJsonDocument(obj).toJson(QJsonDocument::Indented));
    }
}

void MainWindow::loadArenaConfig() {
    QFile file(ARENA_FILENAME);
    if (!file.open(QIODevice::ReadOnly)) return;
    QJsonDocument doc = QJsonDocument::fromJson(file.readAll());
    if (!doc.isObject()) return;

    QJsonObject obj = doc.object();
    QJsonArray arr = obj.value("corners_norm").toArray();
    double w = obj.value("width_cm").toDouble(0.0);
    double h = obj.value("height_cm").toDouble(0.0);
    if (arr.size() != 4 || w <= 1.0 || h <= 1.0) return;

    std::array<cv::Point2f, 4> corners;
    for (int i = 0; i < 4; ++i) {
        QJsonArray pt = arr[i].toArray();
        if (pt.size() != 2) return;
        corners[i] = cv::Point2f(static_cast<float>(pt[0].toDouble()),
                                 static_cast<float>(pt[1].toDouble()));
    }
    flockingSim.configureArena(corners, w, h);
}

void MainWindow::saveArenaConfig() {
    QJsonArray arr;
    for (const auto &c : flockingSim.getCornersNorm()) {
        arr.append(QJsonArray{c.x, c.y});
    }
    QJsonObject obj{
        {"corners_norm", arr},
        {"width_cm", flockingSim.getArenaWidthCm()},
        {"height_cm", flockingSim.getArenaHeightCm()}
    };
    QFile file(ARENA_FILENAME);
    if (file.open(QIODevice::WriteOnly)) {
        file.write(QJsonDocument(obj).toJson(QJsonDocument::Indented));
    }
}

void MainWindow::logMessage(const QString &msg) {
    QString timestamp = QDateTime::currentDateTime().toString("HH:mm:ss.zzz");
    logConsole->appendPlainText(QString("[%1] %2").arg(timestamp, msg));
}