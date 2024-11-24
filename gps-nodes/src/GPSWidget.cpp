#include "GPSWidget.h"
#include "subscriber.h"
#include "publisher.h"
#include <QVBoxLayout>

using namespace std::chrono_literals;

GPSWidget::GPSWidget(QWidget *parent)
	: QWidget(parent),
	publisher_(std::make_shared<MinimalPublisher>()),
	subscriber_(std::make_shared<MinimalSubscriber>()) {
    		talkerLabel = new QLabel("Talker Coordinates: Latitude: N/A, Longitude: N/A, Altitude: N/A", this);
    		listenerLabel = new QLabel("Listener Coordinates: Latitude: N/A, Longitude: N/A, Altitude: N/A", this);

    		QVBoxLayout *layout = new QVBoxLayout;
    		layout->addWidget(talkerLabel);
    		layout->addWidget(listenerLabel);
    		setLayout(layout);
    		setWindowTitle("GPS Coordinates Display");
    		resize(400, 200);

   		connect(subscriber_.get(), &MinimalSubscriber::coordinatesUpdated,
		   	this, &GPSWidget::updateListenerCoordinates);
		connect(publisher_.get(), &MinimalPublisher::coordinatesUpdated,
                        this, &GPSWidget::updateTalkerCoordinates);
	}

	void GPSWidget::updateTalkerCoordinates(double latitude, double longitude, double altitude)
	{
    		talkerLabel->setText(QString("Talker Coordinates: Latitude: %1, Longitude: %2, Altitude: %3")
                             .arg(latitude, 0, 'f', 6)
                             .arg(longitude, 0, 'f', 6)
			     .arg(altitude, 0, 'f', 6));
	}

	void GPSWidget::updateListenerCoordinates(double latitude, double longitude, double altitude)
	{
    		listenerLabel->setText(QString("Listener Coordinates: Latitude: %1, Longitude: %2, Altitude: %3")
                               .arg(latitude, 0, 'f', 6)
                               .arg(longitude, 0, 'f', 6)
			       .arg(altitude, 0, 'f', 6));
	}
