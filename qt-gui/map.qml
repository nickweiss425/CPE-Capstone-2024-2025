import QtQuick 2.15
import QtQuick.Controls 2.15
import QtLocation 5.15
import QtPositioning 5.15

Rectangle {
    id: root
    visible: true
    width: 800
    height: 600

    property var currentLocation: QtPositioning.coordinate(0, 0)

    // Store markers and path
    property var markers: []
    property var pathCoordinates: []

    // Component for the marker
    Component {
        id: markerComponent
        Rectangle {
            width: 20
            height: 20
            radius: 10
            color: "red"
            border.color: "black"
        }
    }

    // Component for the path
    Component {
        id: pathComponent
        MapPolyline {
            line.width: 3
            line.color: "blue"
            path: pathCoordinates
        }
    }

    Map {
        id: map
        anchors.fill: parent
        plugin: Plugin {
            name: "mapboxgl"
            PluginParameter { 
                name: "mapbox.access_token"
                value: "YOUR-API-KEY-HERE" 
            }
        }
        center: currentLocation
        zoomLevel: 12

        MouseArea {
            anchors.fill: parent
            onClicked: {
                if (mouse.button === Qt.LeftButton) {
                    var coordinate = map.toCoordinate(Qt.point(mouse.x, mouse.y))
                    addMarker(coordinate)
                }
            }
        }
    }

    // Create and maintain the path line
    Loader {
        id: pathLoader
        sourceComponent: pathComponent
        onLoaded: {
            if (item) {
                map.addMapItem(item)
            }
        }
    }

    // Modified addMarker function
    function addMarker(coordinate) {
        var marker = Qt.createQmlObject(
            'import QtLocation 5.15; MapQuickItem { }',
            map,
            "dynamicMarker"
        )
        
        var markerItem = markerComponent.createObject(null)
        
        marker.coordinate = coordinate
        marker.anchorPoint = Qt.point(markerItem.width/2, markerItem.height/2)
        marker.sourceItem = markerItem
        
        map.addMapItem(marker)
        markers.push(marker)
        
        // Add coordinate to path
        pathCoordinates.push(coordinate)
        // Force update of the polyline
        pathLoader.item.path = pathCoordinates
    }

    // Function to clear all markers and path
    function clearMap() {
        for (var i = 0; i < markers.length; i++) {
            map.removeMapItem(markers[i])
        }
        markers = []
        pathCoordinates = []
        pathLoader.item.path = []
    }
}