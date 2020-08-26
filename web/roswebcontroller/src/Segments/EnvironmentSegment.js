import React, { Component } from 'react';
import PropTypes from 'prop-types';
import { Segment } from 'semantic-ui-react';
import '../Styles/EnvironmentSegment.css';
import { Icon, Table } from 'semantic-ui-react';
import {ros} from '../ros.js';
import ROSLIB from 'roslib';

class EnvironmentSegment extends Component {
    constructor(props) {
        super(props);
        this.state = {
            beltOneStatus: "OFF",
            beltTwoStatus: "OFF",
            proxOneStatus: "FALSE",
            proxTwoStatus: "FALSE",
            gripperStatus: "DISABLED",
        };
        this.gripperTopic = null;
        this.beltOneTopic = null;
        this.beltTwoTopic = null;
        this.proxOneTopic = null;
        this.proxTwoTopic = null;
    }

    updateGripperStatus = (message) => {
        var status = "DISABLED";
        if (message.data.enabled) {
            if (message.data.attached) {
                status = "ATTACHED";
            } else {
                status = "ENABLED";
            }
        }
        this.setState({
            gripperStatus: status
        })
        this.gripperTopic.unsubscribe();
    }

    updateBeltOneStatus = (message) => {
        var status = (message.data.power > 50) ? "ON" : "OFF";
        this.setState({
            beltOneStatus: status
        });
        this.beltOneTopic.unsubscribe();
    }

    updateBeltTwoStatus = (message) => {
        var status = (message.data.power > 50) ? "ON" : "OFF";
        this.setState({
            beltTwoStatus: status
        });
        this.beltTwoTopic.unsubscribe();
    }

    updateProxOneStatus = (message) => {
        this.setState({
            proxOneStatus: (message.data.object_detected) ? "TRUE" : "FALSE"
        });
        this.proxOneTopic.unsubscribe();
    }

    updateProxTwoStatus = (message) => {
        this.setState({
            proxTwoStatus: (message.data.object_detected) ? "TRUE" : "FALSE"
        });
        this.proxTwoTopic.unsubscribe();
    }

    componentDidMount() {
        this.gripperTopic = new ROSLIB.Topic({
            ros: ros, name: '/arm_controller/gripper/state',
            messageType: 'ur5_t2_4230/VacuumGripperState'
        });
        this.gripperTopic.subscribe((message) => this.updateGripperStatus(message));

        this.beltOneTopic = new ROSLIB.Topic({
            ros: ros, name: 'ur5_t2_4230/conveyor_control/in',
            messageType: 'ur5_t2_4230/ConveyorBeltControl'
        });
        this.beltOneTopic.subscribe((message) => this.updateBeltOneStatus(message));
        
        this.beltTwoTopic = new ROSLIB.Topic({
            ros: ros, name: 'ur5_t2_4230/conveyor_control/out',
            messageType: 'ur5_t2_4230/ConveyorBeltControl'
        });
        this.beltTwoTopic.subscribe((message) => this.updateBeltTwoStatus(message));

        this.proxOneTopic = new ROSLIB.Topic({
            ros: ros, name: '/break_beam_in_sensor ',
            messageType: 'ur5_t2_4230/Proximity'
        });
        this.proxOneTopic.subscribe((message) => this.updateProxOneStatus(message));

        this.proxTwoTopic = new ROSLIB.Topic({
            ros: ros, name: '/break_beam_out_sensor ',
            messageType: 'ur5_t2_4230/Proximity'
        });
        this.proxTwoTopic.subscribe((message) => this.updateProxTwoStatus(message));
    }

    componentWillUnmount() {

    }

    render() {
        return (
            <div>
                <Segment.Group raised>
                    <Segment>
                        <div className="environmentHeaderWrapper">
                            <p className="environmentHeader">Environment</p>
                        </div>
                    </Segment>
                    <Segment>
                    <Table celled striped textAlign="center">
                        <Table.Header>
                        <Table.Row>
                            <Table.HeaderCell >Object</Table.HeaderCell>
                            <Table.HeaderCell >State</Table.HeaderCell>
                        </Table.Row>
                        </Table.Header>

                        <Table.Body>
                        <Table.Row>
                            <Table.Cell collapsing>
                            <Icon name='road' /> Conveyor Belt 1
                            </Table.Cell>
                            <Table.Cell>
                                {(this.state.beltOneStatus == "ON") ? 
                                    <p style={{color:"#2185d0",fontWeight:"bold"}}>{this.state.beltOneStatus}</p>
                                    :
                                    <p>{this.state.beltOneStatus}</p>
                                }
                            </Table.Cell>
                        </Table.Row>
                        <Table.Row>
                            <Table.Cell>
                            <Icon name='road' /> Conveyor Belt 1
                            </Table.Cell>
                            <Table.Cell>
                                {(this.state.beltTwoStatus == "ON") ? 
                                    <p style={{color:"#2185d0",fontWeight:"bold"}}>{this.state.beltTwoStatus}</p>
                                    :
                                    <p>{this.state.beltTwoStatus}</p>
                                }
                            </Table.Cell>
                        </Table.Row>
                        <Table.Row>
                            <Table.Cell>
                            <Icon name='lightning' /> Proximity Sensor 1
                            </Table.Cell>
                            <Table.Cell>
                                {(this.state.proxOneStatus == "TRUE") ? 
                                    <p style={{color:"#2185d0",fontWeight:"bold"}}>{this.state.proxOneStatus}</p>
                                    :
                                    <p>{this.state.proxOneStatus}</p>
                                }
                            </Table.Cell>
                        </Table.Row>
                        <Table.Row>
                            <Table.Cell>
                            <Icon name='lightning' /> Proximity Sensor 2
                            </Table.Cell>
                            <Table.Cell>
                                {(this.state.proxTwoStatus == "TRUE") ? 
                                    <p style={{color:"#2185d0",fontWeight:"bold"}}>{this.state.proxTwoStatus}</p>
                                    :
                                    <p>{this.state.proxTwoStatus}</p>
                                }
                            </Table.Cell>
                        </Table.Row>
                        <Table.Row>
                            <Table.Cell>
                            <Icon name='grab' /> Gripper
                            </Table.Cell>
                            <Table.Cell>
                                {(this.state.gripperStatus == "ENABLED") ? 
                                    <p style={{color:"#2185d0",fontWeight:"bold"}}>{this.state.gripperStatus}</p>
                                    :
                                    (this.state.gripperStatus == "ATTACHED") ?
                                    <p style={{color:"#47F698",fontWeight:"bold"}}>{this.state.gripperStatus}</p>
                                    :
                                    <p>{this.state.gripperStatus}</p>
                                }
                            </Table.Cell>
                        </Table.Row>
                        </Table.Body>
                    </Table>
                    </Segment>
                    {/* <Segment>Right</Segment> */}
                </Segment.Group>
            </div>
        );
    }
}

EnvironmentSegment.propTypes = {

};

export default EnvironmentSegment;