// HTML-Textseite
const char HTMLTEXT[] = R"(<!DOCTYPE html>
<html lang="de">
  <head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body {
        background-color: #a9a9a9;
        display: flex;
        flex-flow: column;
        align-items: center;
        }
        form {
        display: flex;
        flex-flow: column;
        align-items: center;
        }
        .flex-container {
        display: flex;
        flex-flow: row;
        max-width: 30em;
        align-items: center;
        }
        @media all and (width <= 20em) {
            .flex-container {
            display: flex;
            flex-flow: column;
            max-width: 45em;
            align-items: center;
            }
        }
        .flex-item {
        border: 1px solid;
        margin: .2em;
        padding: .1em;
        text-align: center;
        <1background: #ffffff;>
        }
        input {
        font-size: 1em;
        background-color: #adff2f;
        }
        
    </style>
    <script type="text/javascript">
    function updateStep(c) {
        if(c.value >=  5000) {
            if(c.step != 100) {
                c.step = 100;
            }
        } else if(c.value >=  500) {
            if(c.step != 10) {
                c.step = 10;
            }
        } else {
                if(c.step != 1) c.step = 1;
        }
    }
    </script>
    </head>
    <body>
        <h2>Move a stepper</h2>
        <h3>UNO R4 WiFi Board</h3>
        <form action="/stepper">
            <div class="flex-container">
                <p class="flex-item">
                <label> 
                    <label> <b>Ramp Values</b> <br><br> </label>
                    <input style="width:6em; background-color:white" max="160000" min="0" name="ramp" type="number" value="%ld" onchange="updateStep(this) " /> <label>&nbsp steps &nbsp<br><br></label>
                    </label>
                    <input name="setRamp" type="submit" value="Set ramp">
                </p>
                <p class="flex-item">
                    <label> <b>Speed Values</b> <br><br> </label>
                    <label>
                    <input style="width:6em; background-color:white" max="50000" min="0" name="speed" type="number" value="%ld" onchange="updateStep(this) " />&nbsp steps/sec&nbsp <br> <br> 
                    </label>
                    <input name="setSpeed" type="submit" value="Set speed">
                    <input name="setSpeedRamp" type="submit" value="Speed+autoramp">                </p>
            </div>
            <label> <h4> Let the stepper run : </h4> </label>
            <div class="flex-container">
                <p class="flex-item">
                <input name="links" type="submit" value="1 rev left">
                <input name="contl" type="submit" value="Cont. left">
                </p>
                <p class="flex-item">
                <input name="rechts" type="submit" value="1 rev. right">
                <input name="contr" type="submit" value="Cont. right">
                </p>
            </div>
            <div class="flex-container">
                <p class="flex-item"><input style="background-color:red" name="stop" type="submit" value="Stopp"></p>
            </div>
        </form>
    </body>
</html>)";
