import React, { Component } from 'react';
import PropTypes from 'prop-types';
import { Segment, Header, Icon } from 'semantic-ui-react';
import '../Styles/VisionSegment.css';
import { Grid } from '@material-ui/core';
import RvizComponent from '../Components/RvizComponent';
import CVComponent from '../Components/CVComponent';
import {ros} from '../ros.js';
import ROSLIB from 'roslib';

class VisionSegment extends Component {
    constructor(props) {
        super(props);
        this.state = {
            rviz_src: null,
            cv_src: null,
        };
        this.rviz_topic = null;
        this.cv_topic = null;
    }

    updateRvizSrc = (message) => {
        this.setState({
            rviz_src: "data:image/jpg;base64," + message.data,
        });
    }
    
    updateCVSrc = (message) => {
        this.setState({
            cv_src: "data:image/jpg;base64," + message.data,
        });
    }

    componentDidMount() {
        this.rviz_topic = new ROSLIB.Topic({
            ros: ros, name: '/camera/color/image_raw/compressed',
            messageType: 'sensor_msgs/CompressedImage'
        });
        this.rviz_topic.subscribe((message) => this.updateRvizSrc(message));
        this.cv_topic = new ROSLIB.Topic({
            ros: ros, name: '/vision/sent_image',
            messageType: 'sensor_msgs/CompressedImage'
        });
        this.cv_topic.subscribe((message) => this.updateCVSrc(message));
    }

    componentWillUnmount() {

    }

    render() {
        return (
            <div>
                <Segment.Group raised>
                    <Segment>
                        <div className="visionHeaderWrapper">
                            <p className="visionHeader">Vision</p>
                        </div>
                    </Segment>
                    <Segment>
                        <Grid container alignItems="center" justify="center" spacing={2}>
                            <Grid item xs={6}>
                                <Segment>
                                    {(this.state.rviz_src == null) ?
                                        <div style={{
                                            display:"flex",
                                            width:"100%",
                                            height:"300px",
                                            alignItems: "center",
                                            justifyContent:"center",
                                            }}><Icon name="video" size="normal" style={{transform: "translate(-2px,-2px)"}}/><p>Camera Feed</p></div>
                                        :  
                                        <RvizComponent src={this.state.rviz_src}/>
                                    }
                                </Segment>
                            </Grid>
                            <Grid item xs={6}>
                                <Segment>
                                    {(this.state.rviz_src == null) ?
                                        <div style={{
                                            display:"flex",
                                            width:"100%",
                                            height:"300px",
                                            alignItems: "center",
                                            justifyContent:"center",
                                            }}>
                                            <Icon name="video" size="normal" style={{transform: "translate(-2px,-2px)"}}/><p>Computer Vision Feed</p>
                                        </div>
                                        :  
                                        <CVComponent src={this.state.cv_src}/>
                                    }
                                </Segment>
                            </Grid>
                        </Grid>
                    </Segment>
                </Segment.Group>
            </div>
        );
    }
}

VisionSegment.propTypes = {

};

export default VisionSegment;