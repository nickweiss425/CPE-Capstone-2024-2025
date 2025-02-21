import QtQuick 2.2
import QtQuick.Controls 2.2
import QtLocation 5.15
import QtPositioning 5.15

Rectangle {
    id: root
    visible: true
    width: 800
    height: 600

    signal waypointAdded(double latitude, double longitude)
    signal waypointSelected(int index)
    signal waypointRemoved(int index)

    property var currentLocation: QtPositioning.coordinate(0, 0)
    property var hoveredMarker: null
    property var markers: []
    property var pathCoordinates: []

    // Component for the marker
    Component {
        id: markerComponent
        Rectangle {
            id: markerRect
            width: 20
            height: 20
            radius: width / 2
            color: "red"
            border.color: "black"
            border.width: 1
            scale: 1.0
            property int waypointIndex: -1
            property bool selected: false
            transformOrigin: Item.Center  // Ensure scaling is centered

            // Smooth animations for all transitions
            Behavior on scale { NumberAnimation { duration: 150 } }
            Behavior on border.width { NumberAnimation { duration: 150 } }
            Behavior on border.color { ColorAnimation { duration: 150 } }

            states: [
                State {
                    name: "hovered"
                    when: root.hoveredMarker === parent && !selected
                    PropertyChanges {
                        target: markerRect
                        scale: 1.2
                        border.width: 3
                    }
                },
                State {
                    name: "selected"
                    when: selected
                    PropertyChanges {
                        target: markerRect
                        scale: 1.6
                        border.width: 2.4
                        border.color: "black"
                    }
                },
                State {
                    name: ""
                    when: !selected && root.hoveredMarker !== parent
                    PropertyChanges {
                        target: markerRect
                        scale: 1.0
                        border.width: 1
                        border.color: "black"
                    }
                }
            ]

            transitions: [
                Transition {
                    from: "selected"
                    to: ""
                    NumberAnimation { 
                        properties: "scale,border.width" 
                        duration: 150 
                    }
                    ColorAnimation { 
                        properties: "border.color" 
                        duration: 150 
                    }
                }
            ]
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
                value: "pk.eyJ1IjoibWFuZnJvbXBhYmIiLCJhIjoiY200OXV6dTR1MDB2MTJqcHRib3BtZHcwayJ9.Xb9A69GpWoJQSifEN0qPjQ" 
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
                    selectMarkerAtScreenPosition(Qt.point(mouse.x, mouse.y))
                }
            }

            onPositionChanged: {
                updateHoveredMarker(Qt.point(mouse.x, mouse.y))
            }

            onExited: {
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
        markerItem.waypointIndex = markers.length
        
        marker.coordinate = coordinate
        marker.sourceItem = markerItem

        markerItem.scaleChanged.connect(function() {
            marker.anchorPoint = Qt.point(markerItem.width * markerItem.scale * 0.5, 
                                        markerItem.height * markerItem.scale * 0.5)
        })
        
        marker.anchorPoint = Qt.point(markerItem.width * 0.5, markerItem.height * 0.5)

        map.addMapItem(marker)
        markers.push({item: marker, coordinate: coordinate})
        
        pathCoordinates.push(coordinate)
        pathLoader.item.path = pathCoordinates
        waypointAdded(coordinate.latitude, coordinate.longitude)
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

                waypointRemoved(i)
                break
            }
        }
    }

    function selectMarkerAtScreenPosition(mousePoint) {

        for (var j = 0; j < markers.length; j++) {
            markers[j].item.sourceItem.selected = false
        }

        var found = false

        for (var i = 0; i < markers.length; i++) {
            var marker = markers[i].item
            var markerPoint = map.fromCoordinate(marker.coordinate)
            var markerWidth = marker.sourceItem.width
            var markerHeight = marker.sourceItem.height
            
            if (Math.abs(mousePoint.x - markerPoint.x) <= markerWidth/2 &&
                Math.abs(mousePoint.y - markerPoint.y) <= markerHeight/2) {
                marker.sourceItem.selected = true
                waypointSelected(i)
                found = true
                break
            }
        }

        if (!found) {
            waypointSelected(-1)
        }
    }

    function updateWaypointPosition(index, coordinate) {
        for (var i = 0; i < markers.length; i++) {
            if (markers[i].item.sourceItem.waypointIndex === index) {
                markers[i].item.coordinate = coordinate
                markers[i].coordinate = coordinate
                pathCoordinates[i] = coordinate
                pathLoader.item.path = pathCoordinates
                break
            }
        }
    }
}