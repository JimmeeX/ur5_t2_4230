import React, { Component } from 'react';
import PropTypes from 'prop-types';
import { Segment } from 'semantic-ui-react';
import '../Styles/VisionSegment.css';


class VisionSegment extends Component {
    constructor(props) {
        super(props);

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
                        <div className="visionHeaderWrapper">
                            <p className="visionHeader">Vision</p>
                        </div>
                    </Segment>
                    <Segment>Middle</Segment>
                    <Segment>Right</Segment>
                </Segment.Group>
            </div>
        );
    }
}

VisionSegment.propTypes = {

};

export default VisionSegment;