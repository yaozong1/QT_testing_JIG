import QtQuick 2.15
import QtQuick.Controls 2.15


Item {
    property alias buttonFlat: button.flat
    Rectangle {
        id: rectangle
        color: "#2c313c"
        anchors.fill: parent

        Rectangle {
            id: rectangle1
            x: 50
            y: 66
            width: 704
            height: 66
            color: "#495062"
            radius: 16
            layer.wrapMode: ShaderEffectSource.ClampToEdge

            TextField {
                background: Rectangle{
                    id: textcolor
                    color: "#2b2f37"
                    radius: 8

                }
                id: textField
                x: 24
                y: 13
                width: 406
                height: 40
                visible: true
                color: "#02203f"
                horizontalAlignment: Text.AlignLeft
                layer.enabled: false
                hoverEnabled: true
                placeholderTextColor: "#f91f6fc0"
                placeholderText: qsTr("Type your name")


            }

            Button {
                id: button
                x: 460
                y: 13
                width: 155
                height: 40
                text: qsTr("Change Name")
                highlighted: false
                display: AbstractButton.TextBesideIcon
                font.pointSize: 8
                topPadding: 8
                layer.textureSize.width: 1
                layer.effect: button
                layer.samples: 2
                transformOrigin: Item.Center
                wheelEnabled: false
                flat: false
                font.bold: true
            }

            Switch {
                id: switch1
                x: 629
                y: 13
                visible: true
                text: qsTr("Switch")
                checkable: true
                autoRepeat: false
                autoExclusive: false
                checked: false
            }
        }
    }

}

/*##^##
Designer {
    D{i:0;autoSize:true;height:480;width:640}
}
##^##*/
