git {
  "startPoint": {
    "x": 55.0,
    "y": 9.0,
    "heading": "constant",
    "degrees": 90,
    "locked": false
  },
  "lines": [
    {
      "id": "p-leave",
      "name": "1 - Leave to shoot A",
      "endPoint": {
        "x": 55.0,
        "y": 28.0,
        "heading": "linear",
        "startDeg": 90,
        "endDeg": 84.6
      },
      "controlPoints": [],
      "color": "#22c55e",
      "speed": 1,
      "eventMarkers": []
    },
    {
      "id": "p-to-b",
      "name": "2 - To shoot B",
      "endPoint": {
        "x": 30.0,
        "y": 62.0,
        "heading": "linear",
        "startDeg": 84.6,
        "endDeg": 41.3
      },
      "controlPoints": [
        {
          "x": 30.0,
          "y": 28.0
        }
      ],
      "color": "#38bdf8",
      "speed": 1,
      "eventMarkers": []
    },
    {
      "id": "p-flower",
      "name": "3 - To red flower",
      "endPoint": {
        "x": 15.6,
        "y": 47.166666666666664,
        "heading": "linear",
        "startDeg": 41.3,
        "endDeg": 180
      },
      "controlPoints": [],
      "color": "#f97316",
      "speed": 1,
      "eventMarkers": []
    },
    {
      "id": "p-backoff",
      "name": "4 - Back off the flower",
      "endPoint": {
        "x": 30.0,
        "y": 47.166666666666664,
        "heading": "constant",
        "degrees": 180
      },
      "controlPoints": [],
      "color": "#f97316",
      "speed": 1,
      "eventMarkers": []
    },
    {
      "id": "p-to-a",
      "name": "5 - Back to shoot A",
      "endPoint": {
        "x": 55.0,
        "y": 28.0,
        "heading": "linear",
        "startDeg": 180,
        "endDeg": 84.6
      },
      "controlPoints": [],
      "color": "#22c55e",
      "speed": 1,
      "eventMarkers": []
    },
    {
      "id": "p-park",
      "name": "6 - Park in loading zone",
      "endPoint": {
        "x": 17.0,
        "y": 106.0,
        "heading": "linear",
        "startDeg": 84.6,
        "endDeg": 90
      },
      "controlPoints": [
        {
          "x": 30.0,
          "y": 26.0
        },
        {
          "x": 22.0,
          "y": 64.0
        }
      ],
      "color": "#a855f7",
      "speed": 1,
      "eventMarkers": []
    },
    {
      "id": "p-park-direct",
      "name": "6b - Park from the flower",
      "endPoint": {
        "x": 17.0,
        "y": 106.0,
        "heading": "linear",
        "startDeg": 180,
        "endDeg": 90
      },
      "controlPoints": [
        {
          "x": 26.0,
          "y": 78.0
        }
      ],
      "color": "#a855f7",
      "speed": 1,
      "eventMarkers": []
    }
  ],
  "sequence": [
    {
      "kind": "path",
      "lineId": "p-leave"
    },
    {
      "kind": "event",
      "id": "e-shoot-1",
      "name": "Shoot",
      "durationMs": 1000,
      "durationExpression": "shootMs"
    },
    {
      "kind": "wait",
      "id": "w-settle",
      "name": "Let tip-1 dump settle",
      "durationMs": 800
    },
    {
      "kind": "pollen",
      "id": "pollen-spill",
      "name": "Tip-1 spill",
      "targetX": 60.0,
      "targetY": 60.0,
      "zoneRadius": 9,
      "minBalls": 1,
      "timeoutMs": 1200,
      "expectedSearchMs": 300,
      "onTimeout": "skip",
      "standoffInches": 10,
      "retargetInches": 3,
      "approachSpeed": 0.6,
      "intakeDriveInches": 6,
      "intakeMs": 500,
      "intakeEvent": "Intake",
      "returnToStart": true
    },
    {
      "kind": "path",
      "lineId": "p-to-b"
    },
    {
      "kind": "event",
      "id": "e-shoot-2",
      "name": "Shoot",
      "durationMs": 1000,
      "durationExpression": "shootMs"
    },
    {
      "kind": "path",
      "lineId": "p-flower"
    },
    {
      "kind": "event",
      "id": "e-knock",
      "name": "Flower knock",
      "durationMs": 800
    },
    {
      "kind": "path",
      "lineId": "p-backoff"
    },
    {
      "kind": "pollen",
      "id": "pollen-flower",
      "name": "Flower pollen",
      "targetX": 9.0,
      "targetY": 47.166666666666664,
      "zoneRadius": 9,
      "minBalls": 1,
      "timeoutMs": 1200,
      "expectedSearchMs": 300,
      "onTimeout": "blind",
      "standoffInches": 10,
      "retargetInches": 3,
      "approachSpeed": 0.6,
      "intakeDriveInches": 3.5,
      "intakeMs": 600,
      "intakeEvent": "Intake",
      "returnToStart": true
    },
    {
      "kind": "conditional",
      "id": "if-third-shot",
      "name": "Third volley, then park",
      "condition": "takeThirdShot",
      "members": [
        {
          "kind": "path",
          "lineId": "p-to-a"
        },
        {
          "kind": "event",
          "id": "e-shoot-3",
          "name": "Shoot",
          "durationMs": 1000,
          "durationExpression": "shootMs"
        },
        {
          "kind": "path",
          "lineId": "p-park"
        }
      ]
    },
    {
      "kind": "conditional",
      "id": "if-park-only",
      "name": "Park only",
      "condition": "!takeThirdShot",
      "members": [
        {
          "kind": "path",
          "lineId": "p-park-direct"
        }
      ]
    }
  ],
  "shapes": [],
  "variables": [
    {
      "id": "var-shoot-ms",
      "type": "number",
      "name": "shootMs",
      "value": 1000
    },
    {
      "id": "var-third-shot",
      "type": "boolean",
      "name": "takeThirdShot",
      "value": true
    }
  ],
  "settings": {
    "rWidth": 18,
    "rHeight": 16.39,
    "maxVelocity": 73.9,
    "maxAcceleration": 30,
    "maxDeceleration": 197.1,
    "aVelocity": 3.141592653589793,
    "safetyMargin": 1,
    "fieldMap": "biobuzz.webp",
    "autoMidline": "red"
  },
  "version": "1.3.0"
}