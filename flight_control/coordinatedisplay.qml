import QtQuick 6.8
import QtQuick.Controls 6.8
import QtQuick.Layouts 6.8

Item {
    // Expose properties for latitude and longitude to be set externally
    property string latitude: ""
    property string longitude: ""

    RowLayout {
        spacing: 10

        ColumnLayout {
            spacing: 5

            Label {
                text: "Latitude"
                font.bold: true
                horizontalAlignment: Text.AlignHCenter
            }

            TextField {
                id: latitudeInput
                placeholderText: "Latitude"
                text: parent.latitude
                readOnly: true
                Layout.preferredWidth: 240
            }
        }

        ColumnLayout {
            spacing: 5

            Label {
                text: "Longitude"
                font.bold: true
                horizontalAlignment: Text.AlignHCenter
            }

            TextField {
                id: longitudeInput
                placeholderText: "Longitude"
                text: parent.longitude
                readOnly: true
                Layout.preferredWidth: 240
            }
        }
    }

    onLatitudeChanged: {
        latitudeInput.text = latitude
    }

    onLongitudeChanged: {
        longitudeInput.text = longitude
    }
}
