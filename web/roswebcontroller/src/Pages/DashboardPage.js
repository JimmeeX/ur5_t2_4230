import React, { Component, useState } from 'react';
import PropTypes from 'prop-types';
import '../Styles/DashboardPage.css';
import iconwhite from '../Static/iconwhite.png';
import { Icon } from 'semantic-ui-react';
import OrderSegment from '../Segments/OrderSegment';
import VisionSegment from '../Segments/VisionSegment';
import KinematicsSegment from '../Segments/KinematicsSegment';
import EnvironmentSegment from '../Segments/EnvironmentSegment';
import Grid from '@material-ui/core/Grid';
import {ros} from '../ros.js';

function DashboardPage(props) {
    const [rosConnStatus,setRosConnStatus] = useState('disconnected');

    ros.on('connection', function() {
        console.log('Connected to websocket server.');
        setRosConnStatus('connected');
    });

    ros.on('error', function(error) {
        console.log('Error connecting to websocket server: ', error);
        setRosConnStatus('error');
    });

    ros.on('close', function() {
        console.log('Connection to websocket server closed.');
        setRosConnStatus('closed');
    });


    return (
        <div className="root">
            {/* Sidebar */}
            <div className="wcsSidebar">
                <div className="sideSquare">
                    <div className="logoWrapper">
                        <img className="logo" src={iconwhite} />
                    </div>
                </div>
                <div className="sidebarRosWrapper">
                    <p className="sidebarRos">
                        ROS WebSocket:
                    </p>
                    <p className="sidebarRos">
                        {(rosConnStatus == 'connected') &&
                            <Icon name="circle" style={{color:"green"}} size="mini"/>
                        }
                        {(rosConnStatus == 'disconnected' || rosConnStatus == 'error') &&
                            <Icon name="circle" style={{color:"red"}} size="mini"/>
                        }
                        {(rosConnStatus == 'closed' || rosConnStatus == 'error') &&
                            <Icon name="circle" style={{color:"orange"}} size="mini"/>
                        }
                        {rosConnStatus}
                    </p>
                </div>
                <div className="sidebarGroupNameWrapper">
                    <p className="sidebarGroupName">
                        MTRN4230
                    </p>
                    <p className="sidebarGroupName">
                        G6
                    </p>
                </div>
            </div>
            <div className="wcsMainContentWrapper">
                <div className="wcsBanner">
                    <div className="bannerHeaderGroupWrapper">
                        <div className="bannerHeaderWrapper">
                            <p className="bannerHeader">WCS</p>
                        </div>
                        <div className="bannerSubHeaderWrapper">
                            <p className="bannerSubHeader">Warehouse Control System</p>
                        </div>
                    </div>
                    <div className="bannerPageHeaderWrapper">
                        <p className="bannerPageHeader">Administrator Console</p>
                    </div>
                    <div className="bannerVersionWrapper">
                        <p className="bannerVersion">v1.0.0</p>
                    </div>
                </div>
                {/* Content Body */}
                <div className="contentBodyWrapper">
                    <Grid container spacing={3}>
                        <Grid item xs={12}>
                            <OrderSegment />
                        </Grid>
                        <Grid item xs={12}>
                            {/* <KinematicsSegment /> */}
                        </Grid>
                        <Grid item xs={6}>
                            {/* <VisionSegment /> */}
                        </Grid>
                        <Grid item xs={6}>
                            {/* <EnvironmentSegment /> */}
                        </Grid>
                    </Grid>
                </div>
            </div>
        </div>
    );
}

export default DashboardPage;