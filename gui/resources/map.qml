import QtQuick 2.2
import QtQuick.Controls 2.2
import QtQuick.Window 2.2
import QtLocation 5.15
import QtPositioning 5.15

Rectangle {
    id: root
    visible: true
    width: 800
    height: 600
    focus: true

    // Signals to handle map -> waypointManager functions
    signal waypointAdded(double latitude, double longitude)
    signal waypointSelected(int index)
    signal waypointRemoved(int index)

    property var currentLocation: QtPositioning.coordinate(0, 0)
    property var dronePosition: QtPositioning.coordinate(0, 0)
    property var hoveredMarker: null
    property var markers: []
    property var numMarkers: 0
    property var pathCoordinates: []

    // Based on incoming coordinates, change the location of the drone and adjust the pathline and rotation
    function updateDronePosition(latitude, longitude) {
        dronePosition = QtPositioning.coordinate(latitude, longitude)
        updatePath()
        updateDroneRotation()
    }

    function onPopWaypoint() {
        markers.shift()
    }

    // Make the drone icon point towards the first coordinate
    function updateDroneRotation() {
        if (markers.length > 0) {
            var target = markers[0].coordinate
            var dx = target.longitude - dronePosition.longitude
            var dy = target.latitude - dronePosition.latitude
            // Convert to degrees, add 90 to account for image orientation, and negate to match map orientation
            var angle = -(Math.atan2(dy, dx) * 180 / Math.PI) + 90
            droneIcon.rotation = angle
        }
    }

    // Component for the marker
    Component {
        id: markerComponent
        Item {
            id: markerWrapper
            width: 20
            height: 20
            property int waypointIndex: -1
            property bool selected: false
            property int markerType: 1 // 1: small circle, 2: big circle, 3: figure eight, 4: square
            property bool set: false // true: green, false: red
            
            // Small Circle
            Rectangle {
                id: smallCircle
                anchors.fill: parent
                radius: width / 2
                color: markerWrapper.set ? "green" : "red" // changes color based on "set" state
                border.color: "black"
                border.width: 1
                visible: markerWrapper.markerType === 1
                scale: markerWrapper.scale
            }
            
            // Big Circle
            Rectangle {
                id: bigCircle
                width: 30
                height: 30
                anchors.centerIn: parent
                radius: width / 2
                color: markerWrapper.set ? "green" : "red"
                border.color: "black"
                border.width: 1
                visible: markerWrapper.markerType === 2
                scale: markerWrapper.scale
            }
            
            // Figure Eight
            Canvas {
                id: figureEight
                width: 26
                height: 26
                anchors.centerIn: parent
                visible: markerWrapper.markerType === 3
                scale: markerWrapper.scale
                
                // Force redraw when set property changes
                onVisibleChanged: requestPaint()
                Component.onCompleted: requestPaint()
                
                // Force redraw when 'set' changes
                Connections {
                    target: markerWrapper
                    function onSetChanged() {
                        figureEight.requestPaint()
                    }
                }
                
                onPaint: {
                    var ctx = getContext("2d");
                    ctx.reset();
                    
                    // Fill color based on set property
                    ctx.fillStyle = markerWrapper.set ? "green" : "red";
                    ctx.strokeStyle = "black";
                    ctx.lineWidth = 1;
                    
                    // Draw figure eight with more padding to avoid clipping
                    var padding = 4;
                    ctx.beginPath();
                    ctx.arc(width/2, (height-padding)/3 + padding/2, (width-padding)/3, 0, Math.PI * 2, false);
                    ctx.arc(width/2, (height-padding)*2/3 + padding/2, (width-padding)/3, 0, Math.PI * 2, false);
                    ctx.fill();
                    ctx.stroke();
                }
            }
            
            // Square
            Rectangle {
                id: square
                width: 20
                height: 20
                anchors.centerIn: parent
                color: markerWrapper.set ? "green" : "red"
                border.color: "black"
                border.width: 1
                visible: markerWrapper.markerType === 4
                scale: markerWrapper.scale
            }
            
            scale: 1.0
            transformOrigin: Item.Center  // Ensure scaling is centered
            
            // Smooth animations for all transitions
            Behavior on scale { NumberAnimation { duration: 150 } }
            
            states: [
                State {
                    name: "hovered"
                    when: root.hoveredMarker === parent && !markerWrapper.selected
                    PropertyChanges {
                        target: markerWrapper
                        scale: 1.1
                    }
                    PropertyChanges {
                        target: smallCircle
                        border.width: 2
                    }
                    PropertyChanges {
                        target: bigCircle
                        border.width: 2
                    }
                    PropertyChanges {
                        target: square
                        border.width: 2
                    }
                },
                State {
                    name: "selected"
                    when: markerWrapper.selected
                    PropertyChanges {
                        target: markerWrapper
                        scale: 1.2
                    }
                    PropertyChanges {
                        target: smallCircle
                        border.width: 2
                        border.color: "black"
                    }
                    PropertyChanges {
                        target: bigCircle
                        border.width: 2
                        border.color: "black"
                    }
                    PropertyChanges {
                        target: square
                        border.width: 2
                        border.color: "black"
                    }
                },
                State {
                    name: ""
                    when: !markerWrapper.selected && root.hoveredMarker !== parent
                    PropertyChanges {
                        target: markerWrapper
                        scale: 1.0
                    }
                    PropertyChanges {
                        target: smallCircle
                        border.width: 1
                        border.color: "black"
                    }
                    PropertyChanges {
                        target: bigCircle
                        border.width: 1
                        border.color: "black"
                    }
                    PropertyChanges {
                        target: square
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
                        properties: "scale" 
                        duration: 150 
                    }
                    NumberAnimation { 
                        target: smallCircle
                        properties: "border.width" 
                        duration: 150 
                    }
                    NumberAnimation { 
                        target: bigCircle
                        properties: "border.width" 
                        duration: 150 
                    }
                    NumberAnimation { 
                        target: square
                        properties: "border.width" 
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

    // deletes the last marker when backspace is pressed
    Keys.onPressed: {
        if (event.key === Qt.Key_Backspace) {
            if (numMarkers > 0) {
                var lastIndex = markers.length - 1
                map.removeMapItem(markers[lastIndex].item)
                markers.pop()
                pathCoordinates.pop()
                pathLoader.item.path = pathCoordinates
                numMarkers--
                waypointRemoved(lastIndex)
            }
        }
    }

    // clears all markers when delete is pressed
    Keys.onDeletePressed: {
        while (markers.length > 0) {
            map.removeMapItem(markers[0].item)
            waypointRemoved(0)
            markers.shift()
            pathCoordinates.shift()
        }
        pathLoader.item.path = pathCoordinates
        numMarkers = 0
        root.hoveredMarker = null
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
            PluginParameter {
                name: "mapboxgl.mapping.cache.directory"
                value: "/tmp/mapbox/cache"
            }
            PluginParameter {
                name: "mapboxgl.mapping.cache.memory"
                value: true
            }
        }
        center: currentLocation
        zoomLevel: 1

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

    // Drone marker
    MapQuickItem {
        id: droneMarker
        coordinate: dronePosition
        anchorPoint.x: 16
        anchorPoint.y: 16
        visible: true
        sourceItem: Item {
            width: 32
            height: 32
            Image {
                id: droneIcon
                source: "qrc:/drone-icon.png"
                width: parent.width
                height: parent.height
                smooth: true
                anchors.centerIn: parent
                rotation: 0
            }
        }
    }

    Component.onCompleted: {
        map.addMapItem(droneMarker)
        map.addMapItem(droneToFirstPoint)
    }

    // Create and maintain the path lines
    Loader {
        id: pathLoader
        sourceComponent: pathComponent
        onLoaded: {
            if (item) {
                map.addMapItem(item)
            }
        }
    }

    // Drone to first waypoint path
    MapPolyline {
        id: droneToFirstPoint
        line.width: 3
        line.color: "blue"
        path: []
    }

    // Update the path from drone icon to first waypoint
    function updatePath() {
        if (markers.length > 0) {
            droneToFirstPoint.path = [dronePosition, markers[0].coordinate]
            map.addMapItem(droneToFirstPoint)
        } else {
            droneToFirstPoint.path = []
        }
    }

    // Create new marker at given coordinate
    function addMarker(coordinate) {

        if (markers.length >= 20) {
            console.log("Maximum number of waypoints reached, 20 allowed")
            return
        }

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
        numMarkers++
        waypointAdded(coordinate.latitude, coordinate.longitude)
    }

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

    // Function to remove marker at given mouse location, if one exists there
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
                numMarkers--
                waypointRemoved(i)
                break
            }
        }
    }

    // Function to select hovered marker based on right click location
    function selectMarkerAtScreenPosition(mousePoint) {
        var found = false
        var selectedIndex = -1

        // First pass - check if we clicked on a marker
        for (var i = 0; i < markers.length; i++) {
            var marker = markers[i].item
            var markerPoint = map.fromCoordinate(marker.coordinate)
            var markerWidth = marker.sourceItem.width
            var markerHeight = marker.sourceItem.height
            
            if (Math.abs(mousePoint.x - markerPoint.x) <= markerWidth/2 &&
                Math.abs(mousePoint.y - markerPoint.y) <= markerHeight/2) {
                selectedIndex = i
                found = true
                break
            }
        }

        // Second pass - update all markers
        for (var j = 0; j < markers.length; j++) {
            markers[j].item.sourceItem.selected = (j === selectedIndex)
        }

        waypointSelected(selectedIndex)
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

    // Set incoming, new waypoint attributes to their shape and color them green
    function setWaypointData(index, shapeType) {
        if (index >= 0 && index < markers.length) {
            var markerItem = markers[index].item.sourceItem
            markerItem.set = true
            markerItem.markerType = shapeType
        }
    }
}
