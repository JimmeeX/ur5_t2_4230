import React, { Component } from 'react';
import PropTypes from 'prop-types';
import { Segment } from 'semantic-ui-react';
import '../Styles/EnvironmentSegment.css';

class EnvironmentSegment extends Component {
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
                        <div className="environmentHeaderWrapper">
                            <p className="environmentHeader">Environment</p>
                        </div>
                    </Segment>
                    <Segment>Middle</Segment>
                    <Segment>Right</Segment>
                </Segment.Group>
            </div>
        );
    }
}

EnvironmentSegment.propTypes = {

};

export default EnvironmentSegment;