import React, { Component } from 'react';
import { Segment, Icon, Button, Header, Dropdown, List } from 'semantic-ui-react'
import PropTypes from 'prop-types';
import '../Styles/OrderSegment.css';
import Grid from '@material-ui/core/Grid';
import { makeStyles } from '@material-ui/core/styles';
import InputLabel from '@material-ui/core/InputLabel';
import MenuItem from '@material-ui/core/MenuItem';
import FormHelperText from '@material-ui/core/FormHelperText';
import FormControl from '@material-ui/core/FormControl';
import Select from '@material-ui/core/Select';
import {ros} from '../ros.js';
import ROSLIB from 'roslib';

const shapeIconMap = {
    "square" : "square",
    "circle" : "circle",
    "triangle" : "triangle up",
    "none" : "ban"
}

const colourMap = {
    "red" : "#F35253",
    "blue" : "#2185D0",
    "green" : "#47F698",
    "none" : "#000000"
}

class OrderSegment extends Component {
    constructor(props) {
        super(props);
        this.state = {
            newOrderQty: 1,
            newOrderColour: 'none',
            newOrderShape: 'none',
            orderQueue: [],
            orderInPrep: [],
            ordersCompleted: [],
        };
        this.rosPoll = null;
        this.addOrderServiceClient = null;
        this.getOrderServiceClient = null;
    }

    handleAddToQueueClick = () => {
        // Post order to ROS
        var request = new ROSLIB.ServiceRequest({
            color : this.state.newOrderColour,
            shape : this.state.newOrderShape,
            goal: this.state.newOrderQty,
        });
        this.addOrderServiceClient.callService(request, (result) => {
            console.log('Result for service call on '
              + this.addOrderServiceClient.name
              + ': '
              + result);
        });

    }

    handleColourChange = (event) => {
        this.setState({
            newOrderColour: event.target.value
        });
    }

    handleShapeChange = (event) => {
        this.setState({
            newOrderShape: event.target.value
        });
    }

    handleQtyChange = (event) => {
        this.setState({
            newOrderQty: event.target.value
        });
    }

    componentDidMount() {
        this.addOrderServiceClient = new ROSLIB.Service({
            ros : ros,
            name : 'order_manager/add',
        });

        this.getOrderServiceClient = new ROSLIB.Service({
            ros : ros,
            name : 'order_manager/get',
        });

        this.rosPoll = setInterval(() => {
            var request = new ROSLIB.ServiceRequest({});

            this.getOrderServiceClient.callService(request, (response) => {
                console.log(response);
                console.log(response.orders_queued);
                if (JSON.stringify(response.orders_queued) != JSON.stringify(this.state.orderQueue.length)) {
                    this.setState({
                        orderQueue: response.orders_queued
                    });
                }
                if (JSON.stringify(response.orders_doing) != JSON.stringify(this.state.orderInPrep)) {
                    this.setState({
                        orderInPrep: response.orders_doing
                    });
                }
                if (JSON.stringify(response.orders_done) != JSON.stringify(this.state.ordersCompleted)) {
                    this.setState({
                        ordersCompleted: response.orders_done
                    });
                }
            })
        }, 1000)
    }

    componentWillUnmount() {
        clearInterval(this.rosPoll);
        this.rosPoll = null;
    }

    render() {
        return (
            <div>
                <Segment.Group raised>
                    <Segment>
                        <div className="ordersHeaderWrapper">
                            <p className="ordersHeader">Orders</p>
                        </div>
                    </Segment>
                    <Segment>
                        {/* Body */}
                        <Grid container spacing={3}>
                            <Grid item xs={3}> 
                                {/* Create Segment */}
                                <Segment.Group className="createSegment">
                                    <Segment className="createSegmentComponent">
                                        <p className="createHeader"><Icon name='pencil' size='small' />Create</p>
                                    </Segment>
                                        <Segment className="createSegmentComponent">
                                            <List divided relaxed>
                                                <List.Item>
                                                    <List.Content>
                                                        <Grid container alignItems="center" spacing={2}>
                                                            <Grid item xs={2}>
                                                                <Icon name={shapeIconMap[this.state.newOrderShape]} size='large' verticalAlign='middle' style={{color:colourMap[this.state.newOrderColour]}}/>
                                                            </Grid>
                                                            <Grid item xs={8}>
                                                                <div style={{width:"100%",height:"50%"}}>
                                                                    <p className="orderListItemShape">{this.state.newOrderShape}</p>
                                                                </div>
                                                                <div style={{width:"100%",height:"50%"}}>
                                                                    <p className="orderListItemColour">{this.state.newOrderColour}</p>
                                                                </div>
                                                            </Grid>
                                                            <Grid item xs={2}>
                                                                <p className="orderListItemIndex">Qty: {this.state.newOrderQty}</p>
                                                            </Grid>
                                                        </Grid>
                                                    </List.Content>
                                                </List.Item>
                                            </List>
                                        </Segment>
                                    <Segment className="createSegmentComponent">
                                    <Grid container alignItems="center" spacing={1}>
                                        <Grid item xs={4}>
                                            <FormControl style={{width:"100%"}}>
                                                <InputLabel shrink>
                                                Colour
                                                </InputLabel>
                                                <Select
                                                labelId="colour-select-label"
                                                id="colour-select-placeholder-label"
                                                value={this.state.newObjectColour}
                                                onChange={this.handleColourChange}
                                                displayEmpty
                                                >
                                                <MenuItem value="none">
                                                    <em>None</em>
                                                </MenuItem>
                                                <MenuItem value="red"><Icon name='square' style={{color:'#F35253'}}/>Red</MenuItem>
                                                <MenuItem value="green"><Icon name='square' style={{color:'#47F698'}}/>Green</MenuItem>
                                                <MenuItem value="blue"><Icon name='square' style={{color:'#2185D0'}}/>Blue</MenuItem>
                                                </Select>
                                            </FormControl>
                                        </Grid>
                                        <Grid item xs={4}>
                                            <FormControl style={{width:"100%"}}>
                                                <InputLabel shrink>
                                                Shape
                                                </InputLabel>
                                                <Select
                                                labelId="shape-select-label"
                                                id="shape-select-placeholder-label"
                                                value={this.state.newObjectShape}
                                                onChange={this.handleShapeChange}
                                                displayEmpty
                                                >
                                                <MenuItem value="none">
                                                    <em>None</em>
                                                </MenuItem>
                                                <MenuItem value="square"><Icon name='square' style={{color:'#334150'}}/>Square</MenuItem>
                                                <MenuItem value="circle"><Icon name='circle' style={{color:'#334150'}}/>Circle</MenuItem>
                                                <MenuItem value="triangle"><Icon name='triangle up' style={{color:'#334150'}}/>Triangle</MenuItem>
                                                
                                                </Select>
                                            </FormControl>
                                        </Grid>
                                        <Grid item xs={4}>
                                            <FormControl style={{width:"100%"}}>
                                                <InputLabel shrink>
                                                Qty
                                                </InputLabel>
                                                <Select
                                                labelId="qty-select-label"
                                                id="qty-select-placeholder-label"
                                                value={this.state.newObjectQty}
                                                defaultValue={0}
                                                onChange={this.handleQtyChange}
                                                displayEmpty
                                                >
                                                <MenuItem value={1}>1</MenuItem>
                                                <MenuItem value={2}>2</MenuItem>
                                                <MenuItem value={3}>3</MenuItem>
                                                <MenuItem value={4}>4</MenuItem>
                                                <MenuItem value={5}>5</MenuItem>
                                                <MenuItem value={6}>6</MenuItem>
                                                <MenuItem value={7}>7</MenuItem>
                                                <MenuItem value={8}>8</MenuItem>
                                                <MenuItem value={9}>9</MenuItem>
                                                <MenuItem value={10}>10</MenuItem>
                                                <MenuItem value={11}>11</MenuItem>
                                                <MenuItem value={12}>12</MenuItem>
                                                <MenuItem value={13}>13</MenuItem>
                                                <MenuItem value={14}>14</MenuItem>
                                                <MenuItem value={15}>15</MenuItem>
                                                <MenuItem value={16}>16</MenuItem>
                                                <MenuItem value={17}>17</MenuItem>
                                                <MenuItem value={18}>18</MenuItem>
                                                <MenuItem value={19}>19</MenuItem>
                                                <MenuItem value={20}>20</MenuItem>
                                                </Select>
                                            </FormControl>
                                        </Grid>
                                    </Grid>
                                    </Segment>
                                    <Segment className="createSegmentComponent">
                                        <Button onClick={this.handleAddToQueueClick} fluid primary icon labelPosition='left' size="mini">
                                            <Icon name='clipboard list' />
                                            Add Order
                                        </Button>
                                    </Segment>
                                </Segment.Group>
                            </Grid>
                            <Grid item xs={3}>
                                <Segment.Group>
                                    <Segment>
                                        <p className="queueHeader"><Icon name='clipboard list' size='small' />Queue</p>
                                    </Segment>
                                    <Segment>
                                        {
                                            (this.state.orderQueue.length == 0) ?
                                            <p>None</p>
                                            :      
                                        <List divided relaxed>
                                        {
                                            this.state.orderQueue.map((value,index,arr) => (
                                            <List.Item>
                                                <List.Content>
                                                    <Grid container alignItems="center" spacing={2}>
                                                        <Grid item xs={2}>
                                                            <Icon name={shapeIconMap[value.shape]} size='large' verticalAlign='middle' style={{color:colourMap[value.color]}}/>
                                                        </Grid>
                                                        <Grid item xs={5}>
                                                            <div style={{width:"100%",height:"50%"}}>
                                                                <p className="orderListItemShape">{value.shape}</p>
                                                            </div>
                                                            <div style={{width:"100%",height:"50%"}}>
                                                                <p className="orderListItemColour">{value.color}</p>
                                                            </div>
                                                        </Grid>
                                                        <Grid item xs={5}>
                                                            <div style={{width:"100%",height:"50%"}}>
                                                            <p className="orderListItemIndex">Qty: {value.goal}</p>
                                                            </div>
                                                        </Grid>
                                                    </Grid>
                                                </List.Content>
                                            </List.Item>
                                            ))
                                        }
                                        </List>
                                        }
                                    </Segment>
                                </Segment.Group>
                            </Grid>
                            <Grid item xs={3}>
                                <Segment.Group>
                                    <Segment>
                                        <p className="preparingHeader"><Icon name='cogs' size='small' />Preparing</p>
                                    </Segment>
                                    <Segment>
                                        {
                                            (this.state.orderInPrep.length == 0) ?
                                            <p>None</p>
                                            :      
                                        <List divided relaxed>
                                        {
                                            this.state.orderInPrep.map((value,index,arr) => (
                                            <List.Item>
                                                <List.Content>
                                                    <Grid container alignItems="center" spacing={2}>
                                                        <Grid item xs={2}>
                                                            <Icon name={shapeIconMap[value.shape]} size='large' verticalAlign='middle' style={{color:colourMap[value.color]}}/>
                                                        </Grid>
                                                        <Grid item xs={5}>
                                                            <div style={{width:"100%",height:"50%"}}>
                                                                <p className="orderListItemShape">{value.shape}</p>
                                                            </div>
                                                            <div style={{width:"100%",height:"50%"}}>
                                                                <p className="orderListItemColour">{value.color}</p>
                                                            </div>
                                                        </Grid>
                                                        <Grid item xs={5}>
                                                            <div style={{width:"100%",height:"50%"}}>
                                                            <p className="orderListItemIndex">Target: {value.goal}</p>
                                                            </div>
                                                            <div style={{width:"100%",height:"50%"}}>
                                                                <p className="orderListItemColour">Collected: {value.qty}</p>
                                                            </div>
                                                        </Grid>
                                                    </Grid>
                                                </List.Content>
                                            </List.Item>
                                            ))
                                        }
                                        </List>
                                        }
                                    </Segment>
                                </Segment.Group>
                            </Grid>
                            <Grid item xs={3}>
                                <Segment.Group>
                                    <Segment>
                                        <p className="completedHeader"><Icon name='check' size='small' />Completed</p>
                                    </Segment>
                                    <Segment>
                                        {
                                            (this.state.ordersCompleted.length == 0) ?
                                            <p>None</p>
                                            :      
                                        <List divided relaxed>
                                        {
                                            this.state.ordersCompleted.map((value,index,arr) => (
                                            <List.Item>
                                                <List.Content>
                                                    <Grid container alignItems="center" spacing={2}>
                                                        <Grid item xs={2}>
                                                            <Icon name={shapeIconMap[value.shape]} size='large' verticalAlign='middle' style={{color:colourMap[value.color]}}/>
                                                        </Grid>
                                                        <Grid item xs={5}>
                                                            <div style={{width:"100%",height:"50%"}}>
                                                                <p className="orderListItemShape">{value.shape}</p>
                                                            </div>
                                                            <div style={{width:"100%",height:"50%"}}>
                                                                <p className="orderListItemColour">{value.color}</p>
                                                            </div>
                                                        </Grid>
                                                        <Grid item xs={5}>
                                                            <div style={{width:"100%",height:"50%"}}>
                                                            <p className="orderListItemIndex">Qty: {value.goal} 
                                                            <Icon name='check' size='small' verticalAlign='middle' style={{color:colourMap['green'],paddingLeft:"10px"}}/>
                                                            </p>
                                                            </div>
                                                        </Grid>
                                                    </Grid>
                                                </List.Content>
                                            </List.Item>
                                            ))
                                        }
                                        </List>
                                        }
                                    </Segment>
                                </Segment.Group>
                            </Grid>
                        </Grid>
                    </Segment>
                </Segment.Group>
            </div>
        );
    }
}

OrderSegment.propTypes = {

};

export default OrderSegment;