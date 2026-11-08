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
#include <QNetworkDatagram>
#include <QNetworkInterface>
#include <QShortcut>
#include <QRegularExpression>
#include <QtEndian>

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent) {
    setWindowTitle("PicoGo Control Panel");
    resize(1220, 660);

    loadRobotDatabase();
    setupMenuBar();
    setupUi();

    // 1. Gniazdo nasłuchujące meldunków HELLO (Port UDP 5006)
    udpListenSocket = new QUdpSocket(this);
    if (udpListenSocket->bind(QHostAddress::AnyIPv4, UDP_LISTEN_PORT, QUdpSocket::ShareAddress)) {
        connect(udpListenSocket, &QUdpSocket::readyRead, this, &MainWindow::onUdpReadyRead);
        logMessage(QString("Nasłuch telemetrii UDP uruchomiony na porcie %1.").arg(UDP_LISTEN_PORT));
    } else {
        logMessage(QString("BŁĄD: Nie można otworzyć portu UDP %1!").arg(UDP_LISTEN_PORT));
    }

    // 2. Gniazdo wysyłające rozkazy (Port UDP 5005)
    udpCommandSocket = new QUdpSocket(this);

    // 3. Serwer HTTP dla aktualizacji OTA (Port TCP 8000)
    httpOtaServer = new QTcpServer(this);
    if (httpOtaServer->listen(QHostAddress::AnyIPv4, HTTP_OTA_PORT)) {
        connect(httpOtaServer, &QTcpServer::newConnection, this, &MainWindow::onNewHttpConnection);
        logMessage(QString("Serwer HTTP OTA uruchomiony na porcie TCP %1.").arg(HTTP_OTA_PORT));
    } else {
        logMessage(QString("BŁĄD: Port TCP %1 jest zajęty!").arg(HTTP_OTA_PORT));
    }

    // 4. Wbudowany serwer DHCP w tle (Port UDP 67)
    udpDhcpSocket = new QUdpSocket(this);
    connect(udpDhcpSocket, &QUdpSocket::readyRead, this, &MainWindow::onDhcpReadyRead);
    toggleDhcpServer(actDhcpEnable->isChecked());

    // 5. Timer odświeżający statusy ONLINE / OFFLINE co 500 ms
    statusTimer = new QTimer(this);
    connect(statusTimer, &QTimer::timeout, this, &MainWindow::refreshTableStatus);
    statusTimer->start(500);

    // Skrót klawiszowy: Spacja = Natychmiastowy STOP wszystkich robotów
    auto *spaceStop = new QShortcut(QKeySequence(Qt::Key_Space), this);
    connect(spaceStop, &QShortcut::activated, this, &MainWindow::sendStopCommand);
}

void MainWindow::setupMenuBar() {
    QMenu *netMenu = menuBar()->addMenu("⚙ Sieć i DHCP");

    actDhcpEnable = new QAction("Włącz wbudowany serwer DHCP w tle (Pula 192.168.1.10 - .250)", this);
    actDhcpEnable->setCheckable(true);
    actDhcpEnable->setChecked(true);
    connect(actDhcpEnable, &QAction::toggled, this, &MainWindow::toggleDhcpServer);
    netMenu->addAction(actDhcpEnable);

    actMacFilter = new QAction("Filtruj klientów DHCP (Ignoruj telefony – tylko Raspberry Pi i Ubiquiti)", this);
    actMacFilter->setCheckable(true);
    actMacFilter->setChecked(true);
    netMenu->addAction(actMacFilter);
}

void MainWindow::setupUi() {
    auto *centralWidget = new QWidget(this);
    setCentralWidget(centralWidget);
    auto *mainLayout = new QHBoxLayout(centralWidget);
    mainLayout->setContentsMargins(8, 8, 8, 8);
    mainLayout->setSpacing(10);

    // ==================== LEWA KOLUMNA: TABELA FLOTY ====================
    auto *leftLayout = new QVBoxLayout();
    leftLayout->setSpacing(6);
    
    fleetSummaryLabel = new QLabel("Widoczne roboty: 0 | Online: 0 | Zaznaczone: 0 | DHCP: 0");
    fleetSummaryLabel->setStyleSheet("font-size: 13px; font-weight: bold; color: #4fc1ff;");
    leftLayout->addWidget(fleetSummaryLabel);

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
    leftLayout->setStretch(1, 3);
    leftLayout->setStretch(3, 1);

    // ==================== PRAWA KOLUMNA: PANELE STEROWANIA ====================
    auto *rightLayout = new QVBoxLayout();
    rightLayout->setSpacing(6);

    // 1. Wybór celu rozkazów
    auto *targetGroup = new QGroupBox("Cel wysyłanych rozkazów");
    auto *targetLayout = new QVBoxLayout(targetGroup);
    targetLayout->setContentsMargins(8, 6, 8, 6);
    radioTargetSelected = new QRadioButton("Tylko zaznaczone roboty w tabeli (Unicast IP)");
    radioTargetAll = new QRadioButton("Cała flota - wszystkie roboty (Broadcast + Unicast)");
    radioTargetSelected->setChecked(true);
    targetLayout->addWidget(radioTargetSelected);
    targetLayout->addWidget(radioTargetAll);
    rightLayout->addWidget(targetGroup);

    // 2. Szybkie akcje
    auto *actionGroup = new QGroupBox("Sterowanie ruchem i kalibracja");
    auto *actionGrid = new QGridLayout(actionGroup);
    actionGrid->setContentsMargins(8, 6, 8, 6);

    auto *btnStart = new QPushButton("▶ START JAZDY (RUN)");
    btnStart->setMinimumHeight(36);
    btnStart->setStyleSheet("background-color: #1e6f3d; font-weight: bold;");

    auto *btnStop = new QPushButton("⏹ STOP (IDLE) [Spacja]");
    btnStop->setMinimumHeight(36);
    btnStop->setStyleSheet("background-color: #9e2a2b; font-weight: bold;");

    auto *btnCalibrate = new QPushButton("⚙ Kalibruj czujniki linii");
    btnCalibrate->setMinimumHeight(30);

    auto *btnSpin = new QPushButton("↻ Zakręć bączka (SPIN 2s)");
    btnSpin->setMinimumHeight(30);

    auto *btnIdentify = new QPushButton("🔍 Identyfikuj zaznaczone (Krótki obrót 0.5s)");
    btnIdentify->setMinimumHeight(28);

    connect(btnStart, &QPushButton::clicked, this, &MainWindow::sendStartCommand);
    connect(btnStop, &QPushButton::clicked, this, &MainWindow::sendStopCommand);
    connect(btnCalibrate, &QPushButton::clicked, this, &MainWindow::sendCalibrateCommand);
    connect(btnSpin, &QPushButton::clicked, this, &MainWindow::sendSpinCommand);
    connect(btnIdentify, &QPushButton::clicked, this, &MainWindow::sendIdentifyCommand);

    actionGrid->addWidget(btnStart, 0, 0);
    actionGrid->addWidget(btnStop, 0, 1);
    actionGrid->addWidget(btnCalibrate, 1, 0);
    actionGrid->addWidget(btnSpin, 1, 1);
    actionGrid->addWidget(btnIdentify, 2, 0, 1, 2);
    rightLayout->addWidget(actionGroup);

    // 3. Parametry modelu IDM
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

    auto *btnSendParams = new QPushButton("📤 Wyślij zaznaczone parametry");
    btnSendParams->setMinimumHeight(32);
    btnSendParams->setStyleSheet("background-color: #005fb8; font-weight: bold;");
    connect(btnSendParams, &QPushButton::clicked, this, &MainWindow::sendParametersCommand);

    paramGrid->addWidget(chkFreespeed, 0, 0);
    paramGrid->addWidget(spinFreespeed, 0, 1);
    paramGrid->addWidget(chkAcc, 1, 0);
    paramGrid->addWidget(spinAcc, 1, 1);
    paramGrid->addWidget(chkBrake, 2, 0);
    paramGrid->addWidget(spinBrake, 2, 1);
    paramGrid->addWidget(btnSendParams, 3, 0, 1, 2);
    rightLayout->addWidget(paramGroup);

    // 4. Generator korka widmowego
    auto *jamGroup = new QGroupBox("Generator korka widmowego (Impuls hamowania)");
    auto *jamGrid = new QGridLayout(jamGroup);
    jamGrid->setContentsMargins(8, 6, 8, 6);

    jamGrid->addWidget(new QLabel("Zwolnij do prędkości:"), 0, 0);
    spinJamSpeed = new QDoubleSpinBox();
    spinJamSpeed->setRange(0.0, 60.0);
    spinJamSpeed->setValue(12.0);
    jamGrid->addWidget(spinJamSpeed, 0, 1);

    jamGrid->addWidget(new QLabel("Czas trwania [sekundy]:"), 1, 0);
    spinJamDuration = new QDoubleSpinBox();
    spinJamDuration->setRange(0.5, 30.0);
    spinJamDuration->setSingleStep(0.5);
    spinJamDuration->setValue(3.0);
    jamGrid->addWidget(spinJamDuration, 1, 1);

    auto *btnJam = new QPushButton("⚠ Wywołaj zaburzenie na zaznaczonych robotach");
    btnJam->setMinimumHeight(30);
    btnJam->setStyleSheet("background-color: #b5651d; font-weight: bold;");
    connect(btnJam, &QPushButton::clicked, this, &MainWindow::triggerPhantomJam);
    jamGrid->addWidget(btnJam, 2, 0, 1, 2);
    rightLayout->addWidget(jamGroup);

    // 5. Menedżer bezprzewodowego wgrywania kodu (OTA)
    auto *otaGroup = new QGroupBox("Bezprzewodowa aktualizacja kodu (OTA Update)");
    auto *otaLayout = new QVBoxLayout(otaGroup);
    otaLayout->setContentsMargins(8, 6, 8, 6);

    auto *fileRow = new QHBoxLayout();
    editOtaFilePath = new QLineEdit("app.py");
    auto *btnBrowse = new QPushButton("Przeglądaj...");
    connect(btnBrowse, &QPushButton::clicked, this, &MainWindow::browseOtaFile);
    fileRow->addWidget(editOtaFilePath);
    fileRow->addWidget(btnBrowse);
    otaLayout->addLayout(fileRow);

    auto *verRow = new QHBoxLayout();
    chkAutoVersionTag = new QCheckBox("Nadpisz w locie APP_VERSION na:");
    chkAutoVersionTag->setChecked(true);
    editCustomVersionTag = new QLineEdit("4.2");
    verRow->addWidget(chkAutoVersionTag);
    verRow->addWidget(editCustomVersionTag);
    otaLayout->addLayout(verRow);

    auto *btnOtaSend = new QPushButton("🚀 Wgraj wybrany plik przez Wi-Fi (OTA_UPDATE)");
    btnOtaSend->setMinimumHeight(34);
    btnOtaSend->setStyleSheet("background-color: #5c2d91; font-weight: bold;");
    connect(btnOtaSend, &QPushButton::clicked, this, &MainWindow::sendOtaCommand);
    otaLayout->addWidget(btnOtaSend);

    lblOtaStats = new QLabel("Pobrania HTTP OTA w tej sesji: 0");
    otaLayout->addWidget(lblOtaStats);
    rightLayout->addWidget(otaGroup);

    rightLayout->addStretch();

    mainLayout->addLayout(leftLayout, 65);
    mainLayout->addLayout(rightLayout, 35);
}

// ==================== WBUDOWANY SERWER DHCP W TLE ====================

void MainWindow::toggleDhcpServer(bool enabled) {
    if (enabled) {
        if (udpDhcpSocket->bind(QHostAddress::AnyIPv4, DHCP_SERVER_PORT, QUdpSocket::ShareAddress | QUdpSocket::ReuseAddressHint)) {
            logMessage("[DHCP] Serwer DHCP działa w tle na porcie UDP 67 (Pula: 192.168.1.10 - .250).");
        } else {
            logMessage("[DHCP BŁĄD] Port UDP 67 zajęty! Uruchom jako Administrator lub wyłącz inny serwer DHCP.");
            actDhcpEnable->setChecked(false);
        }
    } else {
        udpDhcpSocket->close();
        logMessage("[DHCP] Serwer DHCP został wyłączony z menu górnego.");
    }
}

bool MainWindow::isAllowedDeviceMac(const QString &mac) const {
    if (!actMacFilter->isChecked()) return true;

    // Odrzucanie losowych adresów MAC ze smartfonów (drugi znak: 2, 6, a, e)
    // oraz akceptacja oficjalnych pul OUI dla Raspberry Pi oraz Ubiquiti
    static const QStringList allowedPrefixes = {
        "2c:cf:67", "28:cd:c1", "d8:3a:dd", "dc:a6:32", "e4:5f:01", "b8:27:eb", // Raspberry Pi
        "6c:63:f8", "74:83:c2", "f0:9f:c2", "b4:fb:e4", "24:5a:4c", "80:2a:a8"  // Ubiquiti
    };
    for (const QString &prefix : allowedPrefixes) {
        if (mac.startsWith(prefix, Qt::CaseInsensitive))
            return true;
    }
    return false;
}

QString MainWindow::allocateIpForMac(const QString &mac) {
    if (macToIp.contains(mac)) {
        return macToIp[mac];
    }

    if (mac.startsWith("6c:63:f8") || mac.startsWith("74:83:c2") ||
        mac.startsWith("f0:9f:c2") || mac.startsWith("b4:fb:e4") ||
        mac.startsWith("24:5a:4c") || mac.startsWith("80:2a:a8")) {
        QString apIp = "192.168.1.10";
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
        QString candidate = QString("192.168.1.%1").arg(nextDhcpOctet++);
        if (!macToIp.values().contains(candidate)) {
            assignedIp = candidate;
            break;
        }
    }

    macToIp[mac] = assignedIp;
    saveRobotDatabase();
    refreshTableStatus();
    return assignedIp;
}

void MainWindow::onDhcpReadyRead() {
    while (udpDhcpSocket->hasPendingDatagrams()) {
        QNetworkDatagram datagram = udpDhcpSocket->receiveDatagram();
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

        // Filtr ignorujący telefony i obce urządzenia
        if (!isAllowedDeviceMac(macStr)) {
            continue;
        }

        quint8 dhcpMsgType = 0;
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
            }
            idx += len;
        }

        if (dhcpMsgType != 1 && dhcpMsgType != 3)
            continue;

        QString clientIpStr = allocateIpForMac(macStr);
        quint32 clientIpInt = qToBigEndian(QHostAddress(clientIpStr).toIPv4Address());
        quint32 serverIpInt = qToBigEndian(QHostAddress(SERVER_ETH_IP).toIPv4Address());
        quint32 maskInt     = qToBigEndian(QHostAddress("255.255.255.0").toIPv4Address());

        QByteArray resp(240, 0);
        resp[0] = 2;
        resp[1] = 1;
        resp[2] = 6;
        resp[3] = 0;
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
        resp.append(static_cast<char>(53));
        resp.append(static_cast<char>(1));
        resp.append(static_cast<char>(replyType));

        resp.append(static_cast<char>(54));
        resp.append(static_cast<char>(4));
        resp.append(reinterpret_cast<const char*>(&serverIpInt), 4);

        quint32 leaseTime = qToBigEndian(86400u);
        resp.append(static_cast<char>(51));
        resp.append(static_cast<char>(4));
        resp.append(reinterpret_cast<const char*>(&leaseTime), 4);

        resp.append(static_cast<char>(1));
        resp.append(static_cast<char>(4));
        resp.append(reinterpret_cast<const char*>(&maskInt), 4);

        resp.append(static_cast<char>(3));
        resp.append(static_cast<char>(4));
        resp.append(reinterpret_cast<const char*>(&serverIpInt), 4);

        resp.append(static_cast<char>(255));

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
        if (err.error != QJsonParseError::NoError || !doc.isObject())
            continue;

        QJsonObject obj = doc.object();
        if (obj.value("type").toString() != "HELLO")
            continue;

        QString uid = obj.value("id").toString();
        if (uid.isEmpty())
            continue;

        if (!uidToName.contains(uid)) {
            QString newName = QString("PICO_%1").arg(uidToName.size() + 1);
            uidToName[uid] = newName;
            saveRobotDatabase();
            logMessage(QString("[NOWY ROBOT] Zarejestrowano %1 (UID: %2, IP: %3)")
                           .arg(newName, uid, senderIp));
        }

        RobotInfo &r = fleet[uid];
        QString newVer = obj.value("ver").toString("?");
        if (!r.version.isEmpty() && r.version != newVer) {
            logMessage(QString("[SUKCES OTA] %1 zgłosił się z nową wersją kodu: v%2!")
                           .arg(uidToName[uid], newVer));
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
        if (forceAll || radioTargetAll->isChecked() || it.value().selected) {
            targets.append(it.key());
        }
    }
    return targets;
}

void MainWindow::sendJsonToTargets(const QJsonObject &payload, bool forceAll) {
    QByteArray raw = QJsonDocument(payload).toJson(QJsonDocument::Compact);

    if (forceAll || radioTargetAll->isChecked()) {
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

// ==================== AKCJE PRZYCISKÓW ====================

void MainWindow::sendStartCommand() {
    QJsonObject params{{"mode", "RUN"}};
    sendJsonToTargets(QJsonObject{{"params", params}});
}

void MainWindow::sendStopCommand() {
    QJsonObject params{{"mode", "IDLE"}};
    sendJsonToTargets(QJsonObject{{"params", params}}, true);
}

void MainWindow::sendCalibrateCommand() {
    sendJsonToTargets(QJsonObject{{"cmd", "CALIBRATE"}});
}

void MainWindow::sendSpinCommand() {
    sendJsonToTargets(QJsonObject{{"cmd", "SPIN"}, {"duration", 2.0}, {"power", 50}});
}

void MainWindow::sendIdentifyCommand() {
    sendJsonToTargets(QJsonObject{{"cmd", "SPIN"}, {"duration", 0.5}, {"power", 45}});
}

void MainWindow::sendParametersCommand() {
    QJsonObject params;
    if (chkFreespeed->isChecked()) params["freespeed"] = spinFreespeed->value();
    if (chkAcc->isChecked())       params["acc"]       = spinAcc->value();
    if (chkBrake->isChecked())     params["brake"]     = spinBrake->value();

    if (params.isEmpty()) {
        logMessage("Zaznacz przynajmniej jeden parametr (freespeed, acc lub brake)!");
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

// ==================== ZARZĄDZANIE TABELĄ I BAZĄ ====================

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
            if (col != 1)
                item->setFlags(item->flags() & ~Qt::ItemIsEditable);
            item->setTextAlignment(Qt::AlignCenter);
            fleetTable->setItem(row, col, item);
        }
    }

    fleetTable->item(row, 1)->setText(r.name);
    fleetTable->item(row, 2)->setText(r.ip);
    fleetTable->item(row, 3)->setText("v" + r.version);
    fleetTable->item(row, 5)->setText(r.mode);
    fleetTable->item(row, 5)->setForeground(r.mode == "RUN" ? QColor("#4ec9b0") : QColor("#ce9178"));
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
            logMessage(QString("Zmieniono nazwę robota (%1) na: %2").arg(uid, newName));
        }
    }
}

void MainWindow::selectAllRobots() {
    updatingTableProgrammatically = true;
    for (int row = 0; row < fleetTable->rowCount(); ++row) {
        fleetTable->item(row, 0)->setCheckState(Qt::Checked);
        QString uid = fleetTable->item(row, 0)->data(Qt::UserRole).toString();
        fleet[uid].selected = true;
    }
    updatingTableProgrammatically = false;
    refreshTableStatus();
}

void MainWindow::deselectAllRobots() {
    updatingTableProgrammatically = true;
    for (int row = 0; row < fleetTable->rowCount(); ++row) {
        fleetTable->item(row, 0)->setCheckState(Qt::Unchecked);
        QString uid = fleetTable->item(row, 0)->data(Qt::UserRole).toString();
        fleet[uid].selected = false;
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
                macToIp[lit.key()] = lit.value().toString();
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

void MainWindow::logMessage(const QString &msg) {
    QString timestamp = QDateTime::currentDateTime().toString("HH:mm:ss.zzz");
    logConsole->appendPlainText(QString("[%1] %2").arg(timestamp, msg));
}