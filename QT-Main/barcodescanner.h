#ifndef BARCODESCANNER_H
#define BARCODESCANNER_H

#include <QObject>
#include <QNetworkAccessManager>
#include <QNetworkReply>
#include "item.h" // Item 구조체 정의 필요

class BarcodeScanner : public QObject
{
    Q_OBJECT
public:
    explicit BarcodeScanner(QObject *parent = nullptr);

    void fetchItemDetails(const QString& itemId); // 바코드 스캔 시 호출
    void removeItem(int itemId);

signals:
    // UI(PageCart)로 데이터를 넘겨주는 핵심 신호
    void itemFetched(const Item &item, double cartWeight); 
    void fetchFailed(const QString &error);

private slots:
    void onNetworkReply(QNetworkReply *reply);

private:
    QNetworkAccessManager *manager;
};

#endif // BARCODESCANNER_H