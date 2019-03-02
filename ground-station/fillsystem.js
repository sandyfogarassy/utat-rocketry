import React, { Component } from 'react';
let values = require('./test_numbers.json');
let fill = require('./fillsystem2.png')
let transducer = require('./pressure_transducer_values.json')
let gauge = require('./pressure_gauge_values.json')
 
class App extends Component {
  constructor(props) {
    super(props)
    // aquires last reading from JSON
    this.state = { 
      percentage: values[values.length-1],
      backgroundA1: '#00000000',
      backgroundA2: '#00000000',
      backgroundA3: '#00000000',
      backgroundA4: '#00000000',
      backgroundA5: '#00000000',
      backgroundC5: '#00000000',
      backgroundM1: '#00000000',
      pressureTransducer: transducer[transducer.length-1] / 100,
      pressureGauge: gauge[gauge.length-1] / 100,
      fileMillisecondName: '',
      fileMillisecond: ''
     }

     this.handleClickA1 = this.handleClickA1.bind(this);
     this.handleClickA2 = this.handleClickA2.bind(this);
     this.handleClickA3 = this.handleClickA3.bind(this);
     this.handleClickA4 = this.handleClickA4.bind(this);
     this.handleClickA5 = this.handleClickA5.bind(this);
     this.handleClickC5 = this.handleClickC5.bind(this);
     this.handleClickM1 = this.handleClickM1.bind(this);
  }
  

  render() { 
    return (
      // Renders all elements
      <div>
        <div style={{width:'100%', height:'100%', backgroundImage: `url(${fill})`, position: 'fixed', backgroundSize: 'cover'}}>
          <div className = "cover_for_ox"></div>
          <div className = "hexagon" style = {{opacity: `${this.state.pressureTransducer}`}}></div>
          <div className = 'circle'  style = {{opacity: `${this.state.pressureGauge}`}}></div>
          <button className = "arrow_A1" style={{background: this.state.backgroundA1}}></button>
          <button className = "arrow_A3" style={{background: this.state.backgroundA3}}></button>
          <button className = "arrow_M1" style={{background: this.state.backgroundM1}}></button>
          <button className = "arrow_A2" style={{background: this.state.backgroundA2}}></button>
          <button className = "arrow_C5" style={{background: this.state.backgroundC5}}></button>
          <button className = "arrow_A4" style={{background: this.state.backgroundA4}}></button>
          <button className = "arrow_A5" style={{background: this.state.backgroundA5}}></button>

          <div style={{display: "grid", height: "20%", width: "40%"}}>
            <button style={{gridColumn: "1", gridRow: "2"}} onClick = {this.handleClickM1}>M1</button>
            <button style={{gridColumn: "2", gridRow: "3"}} onClick = {this.handleClickA1}>A1</button>
            <button style={{gridColumn: "3", gridRow: "2"}} onClick = {this.handleClickA3}>A3</button>
            <button style={{gridColumn: "3", gridRow: "4"}} onClick = {this.handleClickA2}>A2</button>
            <button style={{gridColumn: "3", gridRow: "5"}} onClick = {this.handleClickC5}>C5</button>
            <button style={{gridColumn: "4", gridRow: "1"}} onClick = {this.handleClickA4}>A4</button>
            <button style={{gridColumn: "5", gridRow: "1"}} onClick = {this.handleClickA5}>A5</button>
          </div>

          <div style={{display:'grid', height: '12%', width: '20%', position: 'absolute', left: '50%', top: '5%', fontSize: '150%', fontFamily: "Courier New, Courier, monospace"}}>
            <div style={{gridColumn: "1", gridRow: "1"}}>A: Active Points</div>
            <div style={{gridColumn: "1", gridRow: "2"}}>C: Control Points (Passive)</div>
            <div style={{gridColumn: "1", gridRow: "3"}}>M: Manual Points</div>
            <div style={{gridColumn: "1", gridRow: "4"}}>S: Safety Points</div>
          </div>

          <ProgressBar percentage={this.state.percentage} />
        </div>
      </div> 
      );
  }
  handleClickA1() {
    if (this.state.backgroundA1 === "#00000000") {
      return this.setState({backgroundA1: "#0000FF"})
    } 
    
    else {
      return this.setState({backgroundA1: "#00000000"});
    } 
  };

  handleClickA3() {
    if (this.state.backgroundA3 === "#00000000") {
      return this.setState({backgroundA3: "#0000FF"});
    } 
    
    else {
      return this.setState({backgroundA3: "#00000000"});
    } 
  }
  handleClickA2() {
    if (this.state.backgroundA2 === "#00000000") {
      return this.setState({backgroundA2: "#0000FF"});
    } 
    
    else {
      return this.setState({backgroundA2: "#00000000"});
    } 
  };
  handleClickA4() {
    if (this.state.backgroundA4 === "#00000000") {
      return this.setState({backgroundA4: "#0000FF"});
    } 
    
    else {
      return this.setState({backgroundA4: "#00000000"});
    } 
  };
  handleClickA5() {
    if (this.state.backgroundA5 === "#00000000") {
      return this.setState({backgroundA5: "#0000FF"});
    } 
    
    else {
      return this.setState({backgroundA5: "#00000000"});
    } 
  };
  handleClickC5() {
    if (this.state.backgroundC5 === "#00000000") {
      return this.setState({backgroundC5: "#0000FF"});
    } 
    
    else {
      return this.setState({backgroundC5: "#00000000"});
    } 
  };
  handleClickM1() {
    if (this.state.backgroundM1 === "#00000000") {
      return this.setState({backgroundM1: "#0000FF"});
    } 
    
    else {
      return this.setState({backgroundM1: "#00000000"});
    } 
  };

}

// Passing reading as a property to Filler
const ProgressBar = (props) => {
  return (
      <div className="progress-bar" style={{textAlign: 'center'}}>
        <Filler percentage={props.percentage} />
      </div>
    )
}

// Processes reading and sets as a percentage
const Filler = (props) => {
  return <div className="filler" style={{ height: `${props.percentage}%`, fontFamily: "Courier New, Courier, monospace" }}>Ox Tank: {props.percentage}</div>
}




export default App;
