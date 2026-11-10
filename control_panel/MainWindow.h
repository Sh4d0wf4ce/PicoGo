#pragma once

#include <QMainWindow>
#include <QUdpSocket>
#include <QTcpServer>
#include <QTcpSocket>
#include <QTimer>
#include <QTableWidget>
#include <QPlainTextEdit>
#include <QDoubleSpinBox>
#include <QCheckBox>
#include <QRadioButton>
#include <QLineEdit>
#include <QLabel>
#include <QAction>
#include <QMap>
#include <QSet>
#include <QDateTime>
#include <QNetworkInterface>

struct RobotInfo {
    QString uid;
    QString name;
    QString ip;
    QString version;
    QString mode = "IDLE";
    double freespeed = 35.0;
    double acc = 0.5;
    double brake = 0.8;
    double dist = 0.0;
    double spd = 0.0;
    qint64 lastSeenMs = 0;
    bool selected = true;
};

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow() override = default;

private slots:
    // Sieć
    void onUdpReadyRead();
    void onNewHttpConnection();
    void onDhcpReadyRead();
    void toggleDhcpServer(bool enabled);
    void refreshTableStatus();

    // Zaznaczanie w tabeli
    void selectAllRobots();
    void deselectAllRobots();
    void selectOnlineRobots();
    void onTableItemChanged(QTableWidgetItem *item);

    // Komendy sterujące
    void sendStartCommand();
    void sendStopCommand();
    void sendCalibrateCommand();
    void sendSpinCommand();
    void sendIdentifyCommand();
    void sendParametersCommand();
    void triggerPhantomJam();

    // OTA
    void browseOtaFile();
    void sendOtaCommand();

private:
    void setupUi();
    void setupMenuBar();
    void loadRobotDatabase();
    void saveRobotDatabase();
    void updateTableRow(const QString &uid);
    void logMessage(const QString &msg);

    void sendJsonToTargets(const QJsonObject &payload, bool forceAll = false);
    void sendRawUdp(const QByteArray &data, const QString &ip);
    QList<QString> getTargetUids(bool forceAll = false) const;
    QString allocateIpForMac(const QString &mac);
    bool isAllowedDeviceMac(const QString &mac) const;
    bool findServerInterface(QNetworkAddressEntry *entry = nullptr) const;
    void checkNetworkSetup();

    // Stałe portów i adresacji
    static constexpr quint16 UDP_COMMAND_PORT = 5005;
    static constexpr quint16 UDP_LISTEN_PORT  = 5006;
    static constexpr quint16 HTTP_OTA_PORT    = 8000;
    static constexpr quint16 DHCP_SERVER_PORT = 67;
    static constexpr quint16 DHCP_CLIENT_PORT = 68;
    // Cała adresacja sieci robotów wynika z tego jednego prefiksu (/24).
    // Zmieniając go, zmień też ROBOT_IP w skryptach scripts/*_START.bat.
    const QString SUBNET_PREFIX = "192.168.1.";
    const QString SERVER_ETH_IP = SUBNET_PREFIX + "1";
    const QString DB_FILENAME   = "znane_roboty.json";

    // Gniazda sieciowe i timery
    QUdpSocket *udpListenSocket = nullptr;
    QUdpSocket *udpCommandSocket = nullptr;
    QUdpSocket *udpDhcpSocket = nullptr;
    QTcpServer *httpOtaServer = nullptr;
    QTimer *statusTimer = nullptr;

    // Dane floty oraz pula adresów serwera DHCP
    QMap<QString, RobotInfo> fleet;
    QMap<QString, QString> uidToName;
    QMap<QString, QString> macToIp;
    QSet<QString> rejectedMacsLogged;
    int nextDhcpOctet = 10;
    bool updatingTableProgrammatically = false;
    int otaDownloadsCount = 0;
    quint32 commandSeq = 0;

    // Ustawienia serwera DHCP
    QAction *actDhcpEnable = nullptr;
    QAction *actMacFilter  = nullptr;

    // Elementy GUI
    QTableWidget *fleetTable = nullptr;
    QLabel *fleetSummaryLabel = nullptr;
    QRadioButton *radioTargetSelected = nullptr;
    QRadioButton *radioTargetAll = nullptr;

    // Parametry IDM
    QCheckBox *chkFreespeed = nullptr;
    QDoubleSpinBox *spinFreespeed = nullptr;
    QCheckBox *chkAcc = nullptr;
    QDoubleSpinBox *spinAcc = nullptr;
    QCheckBox *chkBrake = nullptr;
    QDoubleSpinBox *spinBrake = nullptr;

    // Generator korka (Zaburzenie)
    QDoubleSpinBox *spinJamSpeed = nullptr;
    QDoubleSpinBox *spinJamDuration = nullptr;

    // Sekcja OTA
    QLineEdit *editOtaFilePath = nullptr;
    QCheckBox *chkAutoVersionTag = nullptr;
    QLineEdit *editCustomVersionTag = nullptr;
    QLabel *lblOtaStats = nullptr;

    // Logi
    QPlainTextEdit *logConsole = nullptr;
};