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
            newOrderObjects: [],
            newObjectColour: 'none',
            newObjectShape: 'none',
            // queuedOrders:
        };
    }

    addItemOnClickHandler = (e) => {
        e.target.blur()
        this.setState((state) =>({
            newOrderObjects:[...state.newOrderObjects,{"colour": state.newObjectColour, "shape": state.newObjectShape}]
        }));
        this.setState({
            newObjectColour: 'none',
            newObjectShape: 'none',
        });
    }

    handleAddToQueueClick = () => {
        
    }

    handleColourChange = (event) => {
        this.setState({
            newObjectColour: event.target.value
        });
    }

    handleShapeChange = (event) => {
        this.setState({
            newObjectShape: event.target.value
        });
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
                                    {
                                    (this.state.newOrderObjects.length == 0) ?
                                        <Segment placeholder className="createSegmentComponent">
                                            <Header icon size="medium" style={{color:"#C4C8D9"}}>
                                            <Icon name='cubes' size='big' style={{color:"#C4C8D9"}}/>
                                            No items have been added to this order.
                                            </Header>
                                        </Segment>
                                        :
                                        <Segment className="createSegmentComponent">
                                            <List divided relaxed>
                                            {
                                                this.state.newOrderObjects.map((object,index,arr) => (
                                                        <List.Item>
                                                            <List.Content>
                                                                <Grid container alignItems="center" spacing={2}>
                                                                    <Grid item xs={2}>
                                                                        {
                                                                            (object.shape == "pentagon") ?
                                                                                <svg 
                                                                                    style={{marginRight:"1px",marginLeft:"2px",paddingTop:"6px"}}
                                                                                    height="18" 
                                                                                    width="18"  
                                                                                    viewBox="0 0 600 600" 
                                                                                    preserveAspectRatio="xMinYMin meet">
                                                                                    <polygon 
                                                                                        points="294,3 585.246118,214.602691 474,556.983037 114,556.983037 2.753882,214.602691" 
                                                                                        fill={colourMap[object.colour]} 
                                                                                        stroke={colourMap[object.colour]} 
                                                                                        stroke-width="1"
                                                                                    />
                                                                                </svg>
                                                                            :
                                                                            <Icon name={shapeIconMap[object.shape]} size='large' verticalAlign='middle' style={{color:colourMap[object.colour]}}/>
                                                                        }
                                                                    </Grid>
                                                                    <Grid item xs={8}>
                                                                        <div style={{width:"100%",height:"50%"}}>
                                                                            <p className="orderListItemShape">{object.shape}</p>
                                                                        </div>
                                                                        <div style={{width:"100%",height:"50%"}}>
                                                                            <p className="orderListItemColour">{object.colour}</p>
                                                                        </div>
                                                                    </Grid>
                                                                    <Grid item xs={2}>
                                                                        <p className="orderListItemIndex">{index + 1}</p>
                                                                    </Grid>
                                                                </Grid>
                                                            </List.Content>
                                                        </List.Item>
                                                ))
                                            }
                                            </List>
                                        </Segment>
                                    }
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
                                                <MenuItem value="pentagon">
                                                    <svg 
                                                        style={{marginRight:"1px",marginLeft:"2px",paddingTop:"6px"}}
                                                        height="18" 
                                                        width="18"  
                                                        viewBox="0 0 600 600" 
                                                        preserveAspectRatio="xMinYMin meet">
                                                        <polygon points="294,3 585.246118,214.602691 474,556.983037 114,556.983037 2.753882,214.602691" fill="#000000" stroke="#000000" stroke-width="1"/>
                                                    </svg>
                                                    Pentagon
                                                </MenuItem>
                                                </Select>
                                            </FormControl>
                                        </Grid>
                                        <Grid item xs={4}>
                                            <Button
                                                icon 
                                                labelPosition='left' 
                                                size="mini"
                                                className="createOrderAddButton"
                                                onClick={this.addItemOnClickHandler}
                                            >
                                                <Icon name='plus' />
                                                Add
                                            </Button>
                                        </Grid>
                                    </Grid>
                                    </Segment>
                                    <Segment className="createSegmentComponent">
                                        <Button onClick={this.handleAddToQueueClick} fluid disabled={(this.state.newOrderObjects.length == 0)} primary icon labelPosition='left' size="mini">
                                            <Icon name='clipboard list' />
                                            Add to Queue
                                        </Button>
                                    </Segment>
                                </Segment.Group>
                            </Grid>
                            <Grid item xs={3}>
                                <Segment.Group>
                                    <Segment>
                                        <p className="queueHeader"><Icon name='clipboard list' size='small' />Queue</p>
                                    </Segment>
                                    <Segment>Middle</Segment>
                                    <Segment>Right</Segment>
                                </Segment.Group>
                            </Grid>
                            <Grid item xs={3}>
                                <Segment.Group>
                                    <Segment>
                                        <p className="preparingHeader"><Icon name='cogs' size='small' />Preparing</p>
                                    </Segment>
                                    <Segment>Middle</Segment>
                                    <Segment>Right</Segment>
                                </Segment.Group>
                            </Grid>
                            <Grid item xs={3}>
                                <Segment.Group>
                                    <Segment>
                                        <p className="completedHeader"><Icon name='check' size='small' />Completed</p>
                                    </Segment>
                                    <Segment>Middle</Segment>
                                    <Segment>Right</Segment>
                                </Segment.Group>
                            </Grid>
                        </Grid>
                    </Segment>
                    <Segment>Right</Segment>
                </Segment.Group>
            </div>
        );
    }
}

OrderSegment.propTypes = {

};

export default OrderSegment;