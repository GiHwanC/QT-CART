#include "barcodescanner.h"
#include <QNetworkRequest>
#include <QUrl>
#include <QDebug>
#include <QJsonDocument>
#include <QJsonObject>
#include <QVariant> // toVariant() 사용을 위해 필수

BarcodeScanner::BarcodeScanner(QObject *parent)
    : QObject(parent),
      manager(new QNetworkAccessManager(this))
{
    connect(manager, &QNetworkAccessManager::finished,
            this, &BarcodeScanner::onNetworkReply);
}

// -------------------------------------------------
// 상품 추가 (스캔) 요청
// -------------------------------------------------
void BarcodeScanner::fetchItemDetails(const QString& itemId)
{
    // 서버 주소 및 포트 확인 (IP: 192.168.123.43, Port: 8000)
    QUrl url(QString("http://192.168.123.43:8000/cart/add/%1").arg(itemId));
    QNetworkRequest request(url);
    request.setHeader(QNetworkRequest::ContentTypeHeader, "application/json");
    
    // POST 요청 전송
    manager->post(request, QByteArray());

    qDebug() << "==================================================";
    qDebug() << "[Scanner] Requesting ADD -> ID:" << itemId;
    qDebug() << "[Scanner] URL:" << url.toString();
}

// -------------------------------------------------
// 상품 제거 요청
// [중요 수정] int -> const QString& 으로 변경 (오버플로우 방지)
// -------------------------------------------------
void BarcodeScanner::removeItem(const QString& itemId)
{
    QUrl url(QString("http://192.168.123.43:8000/cart/remove/%1").arg(itemId));
    QNetworkRequest request(url);
    request.setHeader(QNetworkRequest::ContentTypeHeader, "application/json");

    manager->post(request, QByteArray());

    qDebug() << "==================================================";
    qDebug() << "[Scanner] Requesting REMOVE -> ID:" << itemId;
    qDebug() << "[Scanner] URL:" << url.toString();
}

// -------------------------------------------------
// 서버 응답 처리 (상세 로그 출력 추가)
// -------------------------------------------------
void BarcodeScanner::onNetworkReply(QNetworkReply *reply)
{
    // 1. 네트워크 에러 체크
    if (reply->error() != QNetworkReply::NoError) {
        qDebug() << "[Scanner] Network Error:" << reply->errorString();
        emit fetchFailed(reply->errorString());
        reply->deleteLater();
        return;
    }

    // 2. 데이터 수신 및 원본 로그 출력
    QByteArray data = reply->readAll();
    
    // [DEBUG] 서버에서 온 원본 JSON 데이터를 모두 출력합니다.
    qDebug() << "[Scanner] Raw Response Data:" << data;

    QJsonDocument doc = QJsonDocument::fromJson(data);
    if (!doc.isObject()) {
        qDebug() << "[Scanner] Invalid JSON response (Not an object)";
        reply->deleteLater();
        return;
    }

    QJsonObject obj = doc.object();
    QString action = obj["action"].toString(); 

    qDebug() << "[Scanner] Parsed Action:" << action;

    // 3. 데이터 처리 ("add" 액션)
    if (action == "add") {
        Item item;

        // (1) 상품명 파싱
        if (obj.contains("item")) {
            item.name = obj["item"].toString();
        } else if (obj.contains("name")) {
            item.name = obj["name"].toString();
        } else {
            item.name = "알수없음";
        }

        // (2) ID 파싱 (안전하게 문자열로 변환)
        // [중요] toVariant().toString()을 사용하여 숫자/문자 상관없이 String으로 받음
        if (obj.contains("id")) {
            item.id = obj["id"].toVariant().toString(); 
        } else {
            item.id = ""; 
        }

        // (3) 가격 파싱
        if (obj.contains("price")) {
            item.price = obj["price"].toDouble();
        } else {
            item.price = 0.0; 
        }

        // (4) 무게 파싱
        double expectedWeight = 0.0;
        if (obj.contains("expected_weight")) {
             expectedWeight = obj["expected_weight"].toDouble();
             item.weight = expectedWeight;
        }

        // [DEBUG] 파싱된 상세 결과 출력
        qDebug() << "------------------------------------------";
        qDebug() << "[Scanner] Item Parsed Result:";
        qDebug() << " - Name  :" << item.name;
        qDebug() << " - ID    :" << item.id;
        qDebug() << " - Price :" << item.price;
        qDebug() << " - Weight:" << item.weight;
        qDebug() << "------------------------------------------";

        // UI 갱신 신호 전송
        emit itemFetched(item, expectedWeight);
    }
    else if (action == "remove") {
        // [DEBUG] 삭제 성공 로그
        qDebug() << "[Scanner] Remove confirmed by Server.";
    }
    else {
        qDebug() << "[Scanner] Unknown action received.";
    }

    reply->deleteLater();
}