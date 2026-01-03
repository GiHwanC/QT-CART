#include "barcodescanner.h"
#include <QNetworkRequest>
#include <QUrl>
#include <QDebug>
#include <QJsonDocument>
#include <QJsonObject>

BarcodeScanner::BarcodeScanner(QObject *parent)
    : QObject(parent),
      manager(new QNetworkAccessManager(this))
{
    connect(manager, &QNetworkAccessManager::finished,
            this, &BarcodeScanner::onNetworkReply);
}

// -------------------------------------------------
// 상품 추가 (스캔)
// -------------------------------------------------
void BarcodeScanner::fetchItemDetails(const QString& itemId)
{
    // 서버 주소 확인
    QUrl url(QString("http://192.168.123.43:8000/cart/add/%1").arg(itemId));
    QNetworkRequest request(url);
    request.setHeader(QNetworkRequest::ContentTypeHeader, "application/json");
    
    // POST 요청
    manager->post(request, QByteArray());

    qDebug() << "[Scanner] Requesting ADD Item ID:" << itemId;
}

// -------------------------------------------------
// 상품 제거
// -------------------------------------------------
void BarcodeScanner::removeItem(int itemId)
{
    QUrl url(QString("http://192.168.123.43:8000/cart/remove/%1").arg(itemId));
    QNetworkRequest request(url);
    request.setHeader(QNetworkRequest::ContentTypeHeader, "application/json");

    manager->post(request, QByteArray());

    qDebug() << "[Scanner] Requesting REMOVE Item ID:" << itemId;
}

// -------------------------------------------------
// 서버 응답 처리 (핵심 수정 부분)
// -------------------------------------------------
void BarcodeScanner::onNetworkReply(QNetworkReply *reply)
{
    // 1. 에러 체크
    if (reply->error() != QNetworkReply::NoError) {
        qDebug() << "[Scanner] Error:" << reply->errorString();
        emit fetchFailed(reply->errorString());
        reply->deleteLater();
        return;
    }

    // 2. 데이터 읽기
    QByteArray data = reply->readAll();
    QJsonDocument doc = QJsonDocument::fromJson(data);
    
    if (!doc.isObject()) {
        // 서버가 이상한거 보냄 (혹은 빈 응답)
        reply->deleteLater();
        return;
    }

    QJsonObject obj = doc.object();
    QString action = obj["action"].toString(); // "add" or "remove"

    // 3. UI 업데이트를 위해 데이터 파싱 (add 일때만 처리해도 무방)
    if (action == "add") {
        Item item;
        // 서버 응답 키값("item")이 상품명임. 
        // 주의: 현재 서버 응답에는 id, price가 최상위에 없고 내부 로직에 있음.
        // 서버 응답 예시: { "action": "add", "item": "새우깡", "expected_weight": ... }
        // 따라서 이름만으로 UI에서 매칭하거나, 서버가 더 많은 정보를 주도록 해야 함.
        
        // 일단 PageCart가 이름으로 매칭해서 처리하므로 이름만 잘 넘겨줘도 됨.
        item.name = obj["item"].toString(); 
        
        // 만약 서버가 price도 같이 보내주면 item.price = obj["price"].toInt();
        // 현재 서버 코드 기준으로는 price가 안 넘어오는데, 
        // PageCart::addItemByScan 로직이 이름으로 기존 아이템을 찾으므로 괜찮음.
        
        double expectedWeight = obj["expected_weight"].toDouble();

        // ✅ PageCart로 신호 발사! -> 화면 갱신
        emit itemFetched(item, expectedWeight);
    }
    
    // "remove" 액션일 때는 보통 UI가 먼저 반응하고 서버에 통보하는 식이라
    // 여기서 굳이 처리 안 해도 되지만, 필요하면 로그 출력
    else if (action == "remove") {
        qDebug() << "[Scanner] Remove confirmed by server.";
    }

    reply->deleteLater();
}