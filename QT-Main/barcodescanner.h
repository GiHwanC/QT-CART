#ifndef BARCODESCANNER_H
#define BARCODESCANNER_H

#include <QObject>
#include <QNetworkAccessManager>
#include <QNetworkReply>
#include "item.h"

class BarcodeScanner : public QObject
{
    Q_OBJECT
public:
    explicit BarcodeScanner(QObject *parent = nullptr);

    void fetchItemDetails(const QString& barcodeId);

signals:
    void itemFetched(const Item& item, double cart_weight);
    void fetchFailed(const QString& error);
    void requestStop();



private slots:    void onNetworkReply(QNetworkReply *reply);

private:
    QNetworkAccessManager *manager;
    const QString SERVER_BASE_URL = "http://10.10.14.92:8000";
};

#endif // BARCODESCANNER_H
