import QtQuick 2.2
import QtQuick.Controls 2.2
import QtLocation 5.15
import QtPositioning 5.15

Rectangle {
    id: root
    visible: true
    width: 800
    height: 600

    property var currentLocation: QtPositioning.coordinate(35.29115049008509, -120.67617270722539)

    // Store markers and path
    property var markers: []
    property var pathCoordinates: []
    
    // Track the currently hovered marker
    property var hoveredMarker: null

    // Component for the marker
    Component {
        id: markerComponent
        Rectangle {
            id: markerRect
            width: 20
            height: 20
            radius: 10
            color: "red"
            border.color: "black"
            border.width: 1
            scale: 1.0

            // Smooth animations for hover effects
            Behavior on scale { NumberAnimation { duration: 150 } }
            Behavior on border.width { NumberAnimation { duration: 150 } }

            // States for hover effect
            states: State {
                name: "hovered"
                when: root.hoveredMarker === parent
                PropertyChanges {
                    target: markerRect
                    scale: 1.2
                    border.width: 3
                }
            }
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
                value: "" 
            }
        }
        center: currentLocation
        zoomLevel: 12

        MouseArea {
            anchors.fill: parent
            acceptedButtons: Qt.LeftButton | Qt.MiddleButton | Qt.RightButton
            hoverEnabled: true
            
            onClicked: {
                if (mouse.button === Qt.LeftButton) {
                    var coordinate = map.toCoordinate(Qt.point(mouse.x, mouse.y))
                    addMarker(coordinate)
                } else if (mouse.button === Qt.MiddleButton) {
                    removeMarkerAtScreenPosition(Qt.point(mouse.x, mouse.y))
                }
            }
            
            onReleased: {
                if (mouse.button === Qt.RightButton) {
                    var coordinate = map.toCoordinate(Qt.point(mouse.x, mouse.y))
                    focusOnMarker(coordinate)
                }
            }

            onPositionChanged: {
                // Update hover state when mouse moves
                var mousePoint = Qt.point(mouse.x, mouse.y)
                updateHoveredMarker(mousePoint)
            }

            onExited: {
                // Clear hover state when mouse leaves the map
                root.hoveredMarker = null
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
        markers.push({item: marker, coordinate: coordinate})
        
        pathCoordinates.push(coordinate)
        pathLoader.item.path = pathCoordinates
    }

    // Function to update hovered marker
    function updateHoveredMarker(mousePoint) {
        var foundHoveredMarker = null
        
        for (var i = 0; i < markers.length; i++) {
            var marker = markers[i].item
            var markerPoint = map.fromCoordinate(marker.coordinate)
            var markerWidth = marker.sourceItem.width
            var markerHeight = marker.sourceItem.height
            
            if (Math.abs(mousePoint.x - markerPoint.x) <= markerWidth/2 &&
                Math.abs(mousePoint.y - markerPoint.y) <= markerHeight/2) {
                foundHoveredMarker = marker
                break
            }
        }
        
        root.hoveredMarker = foundHoveredMarker
    }

    function removeMarkerAtScreenPosition(mousePoint) {
        for (var i = 0; i < markers.length; i++) {
            var marker = markers[i].item
            var markerPoint = map.fromCoordinate(marker.coordinate)
            var markerWidth = marker.sourceItem.width
            var markerHeight = marker.sourceItem.height
            
            if (Math.abs(mousePoint.x - markerPoint.x) <= markerWidth/2 &&
                Math.abs(mousePoint.y - markerPoint.y) <= markerHeight/2) {
                map.removeMapItem(markers[i].item)
                markers.splice(i, 1)
                pathCoordinates.splice(i, 1)
                pathLoader.item.path = pathCoordinates
                root.hoveredMarker = null  // Clear hover state
                break
            }
        }
    }

    function focusOnMarker(coordinate) {
        for (var i = 0; i < markers.length; i++) {
            var markerPoint = map.fromCoordinate(markers[i].coordinate)
            var mousePoint = map.fromCoordinate(coordinate)
            
            var markerWidth = markers[i].item.sourceItem.width
            var markerHeight = markers[i].item.sourceItem.height
            
            if (Math.abs(mousePoint.x - markerPoint.x) <= markerWidth/2 &&
                Math.abs(mousePoint.y - markerPoint.y) <= markerHeight/2) {
                modifyPanel.visible = true
                modifyPanel.coordinate = markers[i].coordinate
                break
            }
        }
    }

    function clearMap() {
        for (var i = 0; i < markers.length; i++) {
            map.removeMapItem(markers[i].item)
        }
        markers = []
        pathCoordinates = []
        pathLoader.item.path = []
        root.hoveredMarker = null  // Clear hover state
    }

    Rectangle {
        id: modifyPanel
        width: 200
        height: 100
        color: "lightgrey"
        border.color: "black"
        visible: false

        property var coordinate: null

        Column {
            anchors.centerIn: parent
            spacing: 10

            Button {
                text: "Close"
                onClicked: modifyPanel.visible = false
            }
        }
    }
}