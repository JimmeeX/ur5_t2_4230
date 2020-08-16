import React, { Component } from 'react';
import PropTypes from 'prop-types';
import { Segment } from 'semantic-ui-react';
import '../Styles/KinematicsSegment.css';
import Slider from '@material-ui/core/Slider';



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

    componentDidMount() {

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
                        <Slider value={this.base} onChange={this.handleBaseChange} aria-labelledby="continuous-slider" />
                        <Slider value={this.elbow} onChange={this.handleElbowChange} aria-labelledby="continuous-slider" />
                        <Slider value={this.shoulder} onChange={this.handleShoulderChange} aria-labelledby="continuous-slider" />
                        <Slider value={this.wrist1} onChange={this.handleWrist1Change} aria-labelledby="continuous-slider" />
                        <Slider value={this.wrist2} onChange={this.handleWrist2Change} aria-labelledby="continuous-slider" />
                        <Slider value={this.wrist3} onChange={this.handleWrist3Change} aria-labelledby="continuous-slider" />
                    </Segment>
                    <Segment>Right</Segment>
                </Segment.Group>
            </div>
        );
    }
}

KinematicsSegment.propTypes = {

};

export default KinematicsSegment;