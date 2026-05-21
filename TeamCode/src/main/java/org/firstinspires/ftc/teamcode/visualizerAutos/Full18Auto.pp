{
  "startPoint": {
    "x": 21.872355430183354,
    "y": 122.75669957686883,
    "heading": "linear",
    "startDeg": 90,
    "endDeg": 180,
    "locked": false
  },
  "lines": [
    {
      "id": "line-4uta8u25r3",
      "name": "Shoot to Intake",
      "endPoint": {
        "x": 8.101551480959104,
        "y": 58.153032440056414,
        "heading": "linear",
        "startDeg": 142,
        "endDeg": 180,
        "headingCurve": 0.7
      },
      "controlPoints": [
        {
          "x": 90.92383638928067,
          "y": 67.60543018335682
        },
        {
          "x": 53.700634696756,
          "y": 49.04266572637518
        },
        {
          "x": 11.696050775740481,
          "y": 60.121297602256696
        }
      ],
      "color": "#5977C5",
      "speed": 1,
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": "",
      "eventMarkers": [
        {
          "id": "event-mpbfgom5-pqkav9",
          "name": "Shoot",
          "triggerType": "parametric",
          "position": 0.01,
          "triggerMs": 0,
          "poseX": 8.101551480959104,
          "poseY": 58.153032440056414,
          "durationMs": 1800
        },
        {
          "id": "event-mpbfpnxd-zz7t7m",
          "name": "Intake",
          "triggerType": "parametric",
          "position": 0.45,
          "triggerMs": 0,
          "poseX": 8.101551480959104,
          "poseY": 58.153032440056414,
          "durationMs": 3000
        }
      ]
    },
    {
      "id": "mpb80brk-vs9pp3",
      "name": "Intake to Shoot",
      "endPoint": {
        "x": 57.978187292609555,
        "y": 90.15493264741795,
        "heading": "linear",
        "startDeg": 180,
        "endDeg": 180,
        "headingCurve": 1
      },
      "controlPoints": [
        {
          "x": 52.39671000737671,
          "y": 55.79502626729147
        }
      ],
      "color": "#5977C5",
      "speed": 1,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": "",
      "eventMarkers": [
        {
          "id": "event-mpbfrmqw-ygqj16",
          "name": "Shoot",
          "triggerType": "parametric",
          "position": 0.67,
          "triggerMs": 0,
          "poseX": 57.978187292609555,
          "poseY": 90.15493264741795,
          "durationMs": 2200
        }
      ]
    },
    {
      "id": "mpb81da0-zu8x2d",
      "name": "Gate Intake",
      "endPoint": {
        "x": 11.05245972673443,
        "y": 51.90197461212974,
        "heading": "linear",
        "startDeg": 150,
        "endDeg": 150,
        "headingCurve": 1
      },
      "controlPoints": [
        {
          "x": 55.957255213335266,
          "y": 63.27785613540197
        },
        {
          "x": 0,
          "y": 66.3341659005778
        },
        {
          "x": 12.056578798811621,
          "y": 58.852148975330984
        }
      ],
      "color": "#7665DC",
      "speed": 1,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": "",
      "eventMarkers": [
        {
          "id": "event-mpbfvjhy-jvvmpu",
          "name": "Intake",
          "triggerType": "parametric",
          "position": 0.4,
          "triggerMs": 0,
          "poseX": 11.05245972673443,
          "poseY": 51.90197461212974,
          "durationMs": 2500
        }
      ]
    },
    {
      "id": "mpbf6bnh-hwuqbx",
      "name": "Gate Intake to Shoot",
      "endPoint": {
        "x": 57.978187292609555,
        "y": 90.15493264741795,
        "heading": "linear",
        "startDeg": 150,
        "endDeg": 150,
        "headingCurve": 1
      },
      "controlPoints": [
        {
          "x": 40.27575419607846,
          "y": 69.60045901335394
        }
      ],
      "color": "#7665DC",
      "speed": 1,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": "",
      "eventMarkers": [
        {
          "id": "event-mpbfkx2j-2mnv28",
          "name": "Shoot",
          "triggerType": "parametric",
          "position": 0.71,
          "triggerMs": 0,
          "poseX": 57.978187292609555,
          "poseY": 90.15493264741795,
          "durationMs": 1780
        }
      ]
    },
    {
      "id": "mpbfa540-va00o1",
      "name": "Close Spike",
      "endPoint": {
        "x": 14.90535107818544,
        "y": 82.62511265097167,
        "heading": "linear",
        "startDeg": 150,
        "endDeg": 180,
        "headingCurve": 0.25
      },
      "controlPoints": [
        {
          "x": 41.584434057537464,
          "y": 81.24735777705483
        }
      ],
      "color": "#B6B98B",
      "speed": 1,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": "",
      "eventMarkers": [
        {
          "id": "event-mpbg0abf-5vyjyk",
          "name": "Shoot",
          "triggerType": "parametric",
          "position": 0.22,
          "triggerMs": 0,
          "poseX": 14.90535107818544,
          "poseY": 82.62511265097167,
          "durationMs": 400
        },
        {
          "id": "event-mpedybju-fymt6o",
          "name": "Intake",
          "triggerType": "parametric",
          "position": 0.53,
          "triggerMs": 0,
          "poseX": 14.90535107818544,
          "poseY": 82.62511265097167,
          "durationMs": 1700
        }
      ]
    },
    {
      "id": "mpbfbhxd-78y8mk",
      "name": "Shoot and Leave",
      "endPoint": {
        "x": 52.7191753726149,
        "y": 108.50803854611118,
        "heading": "linear",
        "startDeg": 180,
        "endDeg": 180,
        "headingCurve": 1
      },
      "controlPoints": [],
      "color": "#B6B98B",
      "speed": 1,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": "",
      "eventMarkers": [
        {
          "id": "event-mpbfnwm8-by04j4",
          "name": "Shoot",
          "triggerType": "parametric",
          "position": 0.43,
          "triggerMs": 0,
          "poseX": 52.7191753726149,
          "poseY": 108.50803854611118,
          "durationMs": 0
        }
      ]
    }
  ],
  "shapes": [
    {
      "id": "triangle-1",
      "name": "Red Goal",
      "vertices": [
        {
          "x": 141.5,
          "y": 70
        },
        {
          "x": 141.5,
          "y": 141.5
        },
        {
          "x": 120,
          "y": 141.5
        },
        {
          "x": 138,
          "y": 119
        },
        {
          "x": 138,
          "y": 70
        }
      ],
      "color": "#dc2626",
      "fillColor": "#ff6b6b"
    },
    {
      "id": "triangle-2",
      "name": "Blue Goal",
      "vertices": [
        {
          "x": 6,
          "y": 119
        },
        {
          "x": 25,
          "y": 141.5
        },
        {
          "x": 0,
          "y": 141.5
        },
        {
          "x": 0,
          "y": 70
        },
        {
          "x": 6,
          "y": 70
        }
      ],
      "color": "#2563eb",
      "fillColor": "#60a5fa"
    }
  ],
  "sequence": [
    {
      "kind": "path",
      "lineId": "line-4uta8u25r3"
    },
    {
      "kind": "path",
      "lineId": "mpb80brk-vs9pp3"
    },
    {
      "kind": "repeat",
      "id": "mpbf8umw-i76dg3",
      "name": "Gate Intake Repeat",
      "count": 3,
      "lineIds": [
        "mpb81da0-zu8x2d",
        "mpbf6bnh-hwuqbx"
      ],
      "locked": false
    },
    {
      "kind": "path",
      "lineId": "mpbfa540-va00o1"
    },
    {
      "kind": "path",
      "lineId": "mpbfbhxd-78y8mk"
    }
  ],
  "pathChains": [
    {
      "id": "chain-mpb7tbm1-cxpclx",
      "name": "First Intake",
      "color": "#5977C5",
      "lineIds": [
        "line-4uta8u25r3",
        "mpb80brk-vs9pp3"
      ]
    },
    {
      "id": "mpb816a6-l3czg7",
      "name": "Gate Intake",
      "color": "#7665DC",
      "lineIds": [
        "mpb81da0-zu8x2d",
        "mpbf6bnh-hwuqbx"
      ]
    },
    {
      "id": "mpbrnbat-6mhkjg",
      "name": "Final Intake",
      "color": "#B6B98B",
      "lineIds": [
        "mpbfa540-va00o1",
        "mpbfbhxd-78y8mk"
      ]
    }
  ],
  "poseVariables": [],
  "pathVariables": [],
  "numberVariables": [],
  "settings": {
    "xVelocity": 75,
    "yVelocity": 65,
    "aVelocity": 3.141592653589793,
    "kFriction": 0.1,
    "rWidth": 16,
    "rHeight": 16,
    "safetyMargin": 1,
    "maxVelocity": 40,
    "maxAcceleration": 30,
    "maxDeceleration": 30,
    "fieldMap": "decode.webp",
    "robotImage": "/robot.png",
    "theme": "auto",
    "showGhostPaths": false,
    "showOnionLayers": false,
    "onionLayerSpacing": 3,
    "onionColor": "#dc2626",
    "onionNextPointOnly": true,
    "showHeadingArrow": false,
    "headingArrowLength": 50,
    "headingArrowColor": "#ffffff",
    "headingArrowThickness": 2,
    "pathOpacity": 1,
    "showVelocityGradient": false,
    "showEventPins": true,
    "showEventTimeline": true,
    "showAutoCountdown": false,
    "showPathAnnotations": false,
    "showSwerveModules": true
  },
  "version": "1.2.1",
  "timestamp": "2026-05-20T18:22:44.655Z"
}