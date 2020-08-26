import React, { Component } from 'react';
import PropTypes from 'prop-types';
import { Segment } from 'semantic-ui-react';
import '../Styles/KinematicsSegment.css';
import Slider from '@material-ui/core/Slider';
import {ros} from '../ros.js';
import ROSLIB from 'roslib';
import { Grid, Typography } from '@material-ui/core';



class KinematicsSegment extends Component {
    constructor(props) {
        super(props);
        this.state = {
            base: 0,
            elbow: 0,
            shoulder: 0,
            wrist1: 0,
            wrist2: 0,
            wrist3: 0,
        };
        this.jointsTopic = null;
    }

    handleBaseChange = (event) => {

    }

    handleElbowChange = (event) => {

    }

    handleShoulderChange = (event) => {

    }

    handleWrist1Change = (event) => {

    }

    handleWrist2Change = (event) => {

    }

    handleWrist3Change = (event) => {

    }

    updateJoints = (message) => {
        this.setState({
            base: message.desired.positions[0].toFixed(2),
            elbow: message.desired.positions[1].toFixed(2),
            shoulder: message.desired.positions[2].toFixed(2),
            wrist1: message.desired.positions[3].toFixed(2),
            wrist2: message.desired.positions[4].toFixed(2),
            wrist3: message.desired.positions[5].toFixed(2),
        });
    }

    componentDidMount() {
        this.jointsTopic = new ROSLIB.Topic({
            ros: ros, name: '/arm_controller/state',
            messageType: 'control_msgs/JointTrajectoryControllerState'
        });
        this.jointsTopic.subscribe((message) => this.updateJoints(message));
    }

    componentWillUnmount() {

    }

    render() {
        return (
            <div>
                <Segment.Group raised>
                    <Segment>
                        <div className="kinematicsHeaderWrapper">
                            <p className="kinematicsHeader">Kinematics</p>
                        </div>
                    </Segment>
                    <Segment>
                        <Grid container spacing={2}>
                            <Grid item xs={2}>
                                <p style={{paddingTop:"3px"}}>Base:</p>
                            </Grid>
                            <Grid item xs={9}>
                                <Slider style={{color: "#2185d0"}} value={this.state.base} onChange={this.handleBaseChange} aria-labelledby="continuous-slider" min={-3.1416} max={3.1416} />
                            </Grid>
                            <Grid item xs={1}>
                                <p style={{paddingTop:"3px"}}>{this.state.base}</p>
                            </Grid>
                            <Grid item xs={2}>
                                <p style={{paddingTop:"3px"}}>Elbow:</p>
                            </Grid>
                            <Grid item xs={9}>
                                    <Slider style={{color: "#2185d0"}} value={this.state.elbow} onChange={this.handleElbowChange} aria-labelledby="continuous-slider" min={-3.1416} max={3.1416} />
                            </Grid>
                            <Grid item xs={1}>
                                <p style={{paddingTop:"3px"}}>{this.state.elbow}</p>
                            </Grid>
                            <Grid item xs={2}>
                                <p style={{paddingTop:"3px"}}>Shoulder:</p>
                            </Grid>
                            <Grid item xs={9}>
                                    <Slider style={{color: "#2185d0"}} value={this.state.shoulder} onChange={this.handleShoulderChange} aria-labelledby="continuous-slider" min={-3.1416} max={3.1416} />
                            </Grid>
                            <Grid item xs={1}>
                                <p style={{paddingTop:"3px"}}>{this.state.shoulder}</p>
                            </Grid>
                            <Grid item xs={2}>
                                <p style={{paddingTop:"3px"}}>Wrist 1:</p>
                            </Grid>
                            <Grid item xs={9}>
                                    <Slider style={{color: "#2185d0"}} value={this.state.wrist1} onChange={this.handleShoulderChange} aria-labelledby="continuous-slider" min={-3.1416} max={3.1416} />
                            </Grid>
                            <Grid item xs={1}>
                                <p style={{paddingTop:"3px"}}>{this.state.wrist1}</p>
                            </Grid>
                            <Grid item xs={2}>
                                <p style={{paddingTop:"3px"}}>Wrist 2:</p>
                            </Grid>
                            <Grid item xs={9}>
                                    <Slider style={{color: "#2185d0"}} value={this.state.wrist2} onChange={this.handleShoulderChange} aria-labelledby="continuous-slider" min={-3.1416} max={3.1416} />
                            </Grid>
                            <Grid item xs={1}>
                                <p style={{paddingTop:"3px"}}>{this.state.wrist2}</p>
                            </Grid>
                            <Grid item xs={2}>
                                <p style={{paddingTop:"3px"}}>Wrist 3:</p>
                            </Grid>
                            <Grid item xs={9}>
                                    <Slider style={{color: "#2185d0"}} value={this.state.wrist3} onChange={this.handleShoulderChange} aria-labelledby="continuous-slider" min={-3.1416} max={3.1416} />
                            </Grid>
                            <Grid item xs={1}>
                                <p style={{paddingTop:"3px"}}>{this.state.wrist3}</p>
                            </Grid>
                        </Grid>
                    </Segment>
                    {/* <Segment>Right</Segment> */}
                </Segment.Group>
            </div>
        );
    }
}

KinematicsSegment.propTypes = {

};

export default KinematicsSegment;