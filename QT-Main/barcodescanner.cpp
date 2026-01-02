#include "barcodescanner.h"
#include <QNetworkRequest>
#include <QUrl>
#include <QJsonDocument>
#include <QJsonObject>
#include <QDebug>

BarcodeScanner::BarcodeScanner(QObject *parent)
    : QObject{parent}, manager(new QNetworkAccessManager(this))
{
    connect(manager, &QNetworkAccessManager::finished,
            this, &BarcodeScanner::onNetworkReply);
}

void BarcodeScanner::fetchItemDetails(const QString& barcodeId)
{
    QUrl url(QString("%1/cart/add/%2")
             .arg(SERVER_BASE_URL)
             .arg(barcodeId));

    QNetworkRequest request(url);

    // POST (body 없음)
    manager->post(request, QByteArray());

    qDebug() << "Scanning (POST) item ID:" << barcodeId;
}



void BarcodeScanner::onNetworkReply(QNetworkReply *reply)
{
    int statusCode =
        reply->attribute(QNetworkRequest::HttpStatusCodeAttribute).toInt();

    if (reply->error() != QNetworkReply::NoError) {
        emit fetchFailed(QString("네트워크 오류: %1").arg(reply->errorString()));
        reply->deleteLater();
        return;
    }

    if (statusCode >= 400) {
        emit fetchFailed(QString("서버 오류: %1").arg(statusCode));
        reply->deleteLater();
        return;
    }

    QByteArray responseData = reply->readAll();
    qDebug() << "[SCAN] server replied:" << QString::fromUtf8(responseData);


    QJsonParseError parseError;
    QJsonDocument doc = QJsonDocument::fromJson(responseData, &parseError);

    if (parseError.error != QJsonParseError::NoError || !doc.isObject()) {
        emit fetchFailed("JSON 파싱 실패");
        reply->deleteLater();
        return;
    }

    QJsonObject root = doc.object();

    // 1️⃣ item 객체 존재 확인
    if (!root.contains("item") || !root.value("item").isObject()) {
        emit fetchFailed("서버 응답에 item 이 없습니다.");
        reply->deleteLater();
        return;
    }

    QJsonObject obj = root.value("item").toObject();

    Item item;
    item.id     = obj.value("id").toInt();
    item.name   = obj.value("name").toString();
    item.price  = obj.value("price").toDouble();
    item.stock  = obj.value("stock").toInt();
    item.weight = obj.value("weight").toDouble();

    double cartWeight = root.value("cart_weight").toDouble();
    bool weightOk     = root.value("weight_ok").toBool();

    // 2️⃣ 무게 실패 → 정지 요청
    if (!weightOk) {
        emit requestStop();                           // 로봇 정지
        emit fetchFailed("무게가 일치하지 않습니다!"); // 팝업 띄울 메시지
    }


    // 3️⃣ UI/Cart 쪽으로 전달
    emit itemFetched(item, cartWeight);

    reply->deleteLater();
}

void BarcodeScanner::removeItem(int itemId)
{
    QUrl url(QString("%1/cart/remove/%2")
             .arg(SERVER_BASE_URL)
             .arg(itemId));

    QNetworkRequest request(url);

    auto reply = manager->post(request, QByteArray());

    qDebug() << "[REMOVE] request item =" << itemId;

    connect(reply, &QNetworkReply::finished, this, [this, reply]() {

        QByteArray response = reply->readAll();
        qDebug() << "[REMOVE] server replied:" << QString::fromUtf8(response);

        QJsonParseError err;
        QJsonDocument doc = QJsonDocument::fromJson(response, &err);

        if (err.error != QJsonParseError::NoError || !doc.isObject()) {
            emit fetchFailed("REMOVE JSON 파싱 실패");
            reply->deleteLater();
            return;
        }

        QJsonObject root = doc.object();

        double cartWeight = root.value("cart_weight").toDouble();
        bool weightOk     = root.value("weight_ok").toBool();

        
        if (!weightOk) {
            emit requestStop();
        }

        reply->deleteLater();
    });
}
